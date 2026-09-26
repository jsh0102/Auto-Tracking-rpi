// servo — MG90 팬/틸트 서보 PWM 드라이버 (커널 모듈)
//
// pigpio 는 /dev/mem 으로 GPIO 레지스터를 직접 만져 PWM 을 만든다. 커널을
// 우회하므로 root 가 필요하고, 커널은 그 핀이 사용 중인 줄 모른다.
// 이 모듈은 같은 일을 커널 안에서 hrtimer 로 한다.
//
// 지금은 B 단계 — 여기에 비상정지 버튼을 붙였다. 커널이 GPIO 인터럽트로 눌림을
// 즉시 감지하고, /dev/button0 을 read 하며 잠들어 있는 유저 프로세스를 깨운다.
// 정지 판단은 아직 유저스페이스가 한다 (D 에서 커널로 옮긴다).
//
//   echo "pan 1500"    > /dev/servo0    1500us 펄스 (기본 단위)
//   echo "pan 30deg"   > /dev/servo0    30도 -> 커널이 us 로 환산
//   echo "pan off"     > /dev/servo0    펄스 중단 (서보가 힘을 놓는다)
//   cat /dev/servo0                     현재 상태
//   cat /sys/class/servo/servo0/jitter  타이머 지터 통계
//   echo 0 > /sys/class/servo/servo0/jitter   통계 초기화
//
//   read(/dev/button0)  버튼이 눌릴 때까지 잠든다. 깨어나면 한 줄을 돌려준다:
//                         press 3 ktime=123456789012
//                       ktime 은 CLOCK_MONOTONIC 나노초다. 유저의
//                       clock_gettime(CLOCK_MONOTONIC) 과 직접 비교할 수 있다.
//
// 기본 단위를 us 로 둔 이유: 커널은 하드웨어가 쓰는 단위를 받고, 서보마다
// 다른 보정값(config.json 의 min/max_pulse_ms)은 유저스페이스가 갖는다.
// "방법은 커널에, 정책은 유저스페이스에." 각도 입력은 손으로 시험할 때 쓰는
// 편의 기능이다.

// pr_info 등이 찍는 줄 앞에 모듈 이름을 자동으로 붙인다.
// include 보다 먼저 정의해야 효과가 있다 (printk.h 가 이 매크로를 본다).
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/hrtimer.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/kstrtox.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/wait.h>

#define SERVO_CLASS_NAME  "servo"	/* /sys/class/servo */
#define SERVO_DEV_NAME    "servo0"	/* /dev/servo0 */
#define SERVO_WRITE_MAX   31		/* 한 번에 받아들이는 최대 글자 수 */

// gpiochip0 의 base 가 0 이라 BCM 번호를 그대로 쓸 수 있다.
// base 가 512 인 시스템이라면 여기에 512 를 더해야 한다.
#define SERVO_PAN_GPIO    12
#define SERVO_TILT_GPIO   13
#define SERVO_BUTTON_GPIO 17		/* 비상정지 버튼. 내부 풀업, 누르면 LOW */

// 기계 접점은 붙을 때도 떨어질 때도 수십 번 튄다(채터링).
//
// 시간 창("마지막 누름에서 N ms 안이면 무시")으로는 막을 수 없다. 누름과 뗌은
// 사람이 얼마나 오래 쥐고 있느냐에 따라 수백 ms 떨어져 있어서, 시계만 보면
// "눌렀다 뗀 것"과 "두 번 누른 것"이 구분되지 않는다.
//
// 그래서 조건을 시간이 아니라 핀 상태로 바꾼다 — **떼어진 것을 확인해야 다음
// 누름을 받는다.** 떼는 순간에도 튀므로, 엣지가 올 때마다 정착 타이머를 뒤로
// 밀고, 타이머가 울렸다는 것(= 이 시간 동안 엣지가 없었다)을 "안정됐다"의
// 증거로 쓴다. 주기적으로 확인하는 폴링이 아니다.
//
// 하드웨어(RC)로 막지 않은 이유는 신호 경로에 시간 상수를 심으면 우리가 재려는
// 지연에 그것이 섞이기 때문이다. BCM2711 GPIO 에는 하드웨어 디바운스가 없다.
#define SERVO_SETTLE_MS 20

// 한 드라이버가 장치 둘을 담당한다. minor 번호가 그 둘을 가른다.
#define SERVO_MINOR_SERVO  0		/* /dev/servo0  — 펄스 지시, 상태 읽기 */
#define SERVO_MINOR_BUTTON 1		/* /dev/button0 — 눌릴 때까지 대기 */
#define SERVO_MINOR_COUNT  2
#define SERVO_BUTTON_NAME  "button0"

// MG90 서보 규격. config.json 의 실측값과 같은 값이다.
//   min_pulse_ms: 0.5  ->  500us  ->  -90도
//   max_pulse_ms: 2.5  -> 2500us  ->  +90도
// 20ms 주기(50Hz)는 아날로그 서보의 공통 규격이다.
#define SERVO_PERIOD_US     20000
#define SERVO_MIN_PULSE_US  500
#define SERVO_MAX_PULSE_US  2500
#define SERVO_MIN_DEG       (-90)
#define SERVO_MAX_DEG       90

// 지터 히스토그램 구간. 각도 기준으로 끊었다 (1도 = 약 11us).
//   [0] <11us   1도 미만 — 서보가 반응하지 않는 크기
//   [1] <33us   1~3도   — deadzone 0.12 안. 추적 로직이 무시한다
//   [2] <55us   3~5도   — deadzone 을 넘는다
//   [3] <110us  5~10도  — 확실히 보인다
//   [4] 나머지  10도 이상
#define SERVO_JIT_BUCKETS 5
static const u32 servo_jit_edge_ns[SERVO_JIT_BUCKETS - 1] = {
	11000, 33000, 55000, 110000
};
static const char *const servo_jit_label[SERVO_JIT_BUCKETS] = {
	"  <11us (<1도)", " <33us (1~3도)", " <55us (3~5도)",
	"<110us (5~10도)", ">=110us (10도+)"
};

// 이 커널의 ktime.h 에는 ns_to_ktime / ms_to_ktime 만 있고
// us_to_ktime 이 없다. 직접 만든다.
static inline ktime_t servo_us_to_ktime(u64 us)
{
	return ns_to_ktime(us * 1000ULL);
}

static dev_t servo_devno;		/* 커널이 배정해 준 major/minor */
static struct cdev servo_cdev;		/* 이 장치와 함수 표를 묶는 것 */
static struct class *servo_class;	/* /dev 노드를 만들어 달라고 할 때 필요 */
static struct device *servo_device;
static struct device *button_device;

// ───────────────────────── 비상정지 버튼 ─────────────────────────
//
// 폴링은 "왔어?" 를 반복하느라 CPU 를 쓴다. 인터럽트는 하드웨어가 CPU 를 깨우므로
// 평소 비용이 0 이다. 유저 프로세스도 물어보는 대신 대기 큐에서 잠들어 기다린다.
static struct gpio_desc *button_desc;
static int button_irq = -1;
static DECLARE_WAIT_QUEUE_HEAD(button_wq);	/* 유저가 잠들어 기다리는 곳 */
static DEFINE_SPINLOCK(button_lock);		/* 아래 세 값을 보호한다 */
static struct hrtimer button_settle;		/* 신호가 조용해졌는지 재는 타이머 */
static bool button_armed = true;		/* 다음 누름을 받을 준비가 됐나 */
static u64 button_seq;				/* 받아들인 눌림 횟수 */
static u64 button_irq_count;			/* 실제로 들어온 인터럽트 수 */
static u64 button_reject_count;			/* 채터링으로 버린 수 */
static ktime_t button_stamp;			/* 마지막 눌림 시각 (기준시계) */

// /dev/button0 을 연 쪽마다 "어디까지 봤는지" 를 따로 들고 있어야 한다.
struct button_reader {
	u64 seen;
};

// 팬/틸트를 같은 방식으로 다루기 위한 묶음.
struct servo_channel {
	const char *name;		/* echo 로 지정할 이름 */
	unsigned int gpio;		/* BCM 번호 */
	const char *label;		/* 커널 장부에 남을 이름 */
	struct gpio_desc *desc;		/* 핀 손잡이 */

	struct hrtimer timer;		/* 알람시계 */
	spinlock_t lock;		/* 아래 세 값을 보호한다 */
	unsigned int pulse_us;		/* 목표 펄스폭. 0 이면 중단 */
	bool level;			/* 지금 핀이 HIGH 인가 */
	bool running;			/* 펄스를 반복하는 중인가 */

	/* 지터 통계 — 콜백이 예정 시각보다 얼마나 늦게 불렸는가.
	 * 펄스폭은 콜백 타이밍 그 자체라, 이 늦음이 곧 펄스폭 오차다.
	 * 1도 = 약 11us 이므로 여기 숫자를 11로 나누면 각도 오차가 된다. */
	bool stats_skip;		/* 시작 직후 첫 콜백은 빼놓는다 */
	u64 jit_count;
	u64 jit_sum_ns;
	u64 jit_max_ns;
	u64 jit_bucket[SERVO_JIT_BUCKETS];
};

static struct servo_channel servo_ch[] = {
	{ .name = "pan",  .gpio = SERVO_PAN_GPIO,  .label = "servo-pan"  },
	{ .name = "tilt", .gpio = SERVO_TILT_GPIO, .label = "servo-tilt" },
};

static struct servo_channel *servo_find(const char *name)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(servo_ch); i++) {
		if (!strcmp(servo_ch[i].name, name))
			return &servo_ch[i];
	}
	return NULL;
}

// ───────────────────────── 단위 변환 ─────────────────────────
//
// 커널에서는 double/float 를 쓸 수 없다. 커널이 문맥을 바꿀 때 부동소수점
// 레지스터를 저장·복원하지 않기 때문이다. 그래서 전부 정수 산술로 한다.
//
//   펄스폭 = 500 + (각도 + 90) x (2500 - 500) / 180
//
// 나눗셈을 마지막에 두는 게 중요하다. 먼저 나누면 소수점이 버려져 오차가 커진다.
// 그리고 나누기 전에 제수의 절반을 더해 반올림한다. 안 그러면 30deg 를 넣고
// 다시 읽었을 때 29deg 로 보인다 — 정수 나눗셈이 늘 내림이기 때문이다.

static unsigned int servo_deg_to_us(int deg)
{
	const int span_deg = SERVO_MAX_DEG - SERVO_MIN_DEG;

	return SERVO_MIN_PULSE_US +
	       (unsigned int)(((deg - SERVO_MIN_DEG) *
			       (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US) +
			       span_deg / 2) / span_deg);
}

static int servo_us_to_deg(unsigned int us)
{
	const unsigned int span_us = SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US;

	return SERVO_MIN_DEG +
	       (int)(((us - SERVO_MIN_PULSE_US) *
		      (SERVO_MAX_DEG - SERVO_MIN_DEG) +
		      span_us / 2) / span_us);
}

// "1500" / "1500us" / "30deg" / "-45deg" / "off" 를 펄스폭으로 바꾼다.
// off 는 0 으로 돌려준다 (0 = 펄스 중단).
//
// 접미사가 없으면 us 로 본다. 커널의 기본 단위는 하드웨어 단위다.
static int servo_parse_pulse(const char *arg, unsigned int *pulse_us)
{
	char num[16];
	size_t len = strlen(arg);
	bool is_deg = false;
	int value;

	if (!strcmp(arg, "off")) {
		*pulse_us = 0;
		return 0;
	}

	if (len > 3 && !strcmp(arg + len - 3, "deg")) {
		len -= 3;
		is_deg = true;
	} else if (len > 2 && !strcmp(arg + len - 2, "us")) {
		len -= 2;
	}

	if (len == 0 || len >= sizeof(num))
		return -EINVAL;
	memcpy(num, arg, len);
	num[len] = '\0';

	// kstrtoint 는 앞의 '-' 도 처리한다. 숫자가 아니면 오류를 돌려준다.
	if (kstrtoint(num, 10, &value))
		return -EINVAL;

	if (is_deg) {
		if (value < SERVO_MIN_DEG || value > SERVO_MAX_DEG)
			return -ERANGE;
		*pulse_us = servo_deg_to_us(value);
		return 0;
	}

	// 범위 밖 펄스를 주면 서보가 기구적 한계를 계속 밀면서 발열하고 기어가
	// 상한다. 그래서 커널이 막는다.
	if (value < SERVO_MIN_PULSE_US || value > SERVO_MAX_PULSE_US)
		return -ERANGE;

	*pulse_us = (unsigned int)value;
	return 0;
}

// ───────────────────────── 펄스 생성 ─────────────────────────
//
// 알람이 울릴 때마다 핀을 올리거나 내린다. 둘 중 무엇을 할지는 level 로 안다.
//
//         ┌──────┐                              ┌──────┐
// ────────┘      └──────────────────────────────┘      └────
//         ↑      ↑                              ↑
//       올림    내림                          올림
//         ├pulse_us                          
//         ├────── SERVO_PERIOD_US ────────────┤
//
// 초당 100번(50Hz x 2) 불린다. 그래서 여기서는 로그를 찍지 않는다.
// 그리고 이 자리는 잠들 수 없는 자리다 — mutex 가 아니라 spinlock 을 쓰는 이유.
static enum hrtimer_restart servo_tick(struct hrtimer *timer)
{
	// 타이머 포인터로부터 그것을 품고 있는 구조체를 되찾는 커널 관용구.
	struct servo_channel *ch =
		container_of(timer, struct servo_channel, timer);
	unsigned long flags;
	unsigned int pulse;
	u64 next_us;
	s64 late_ns;

	// 이번 콜백이 "불렸어야 할 시각" 과 "실제로 불린 시각" 의 차이.
	// hrtimer_forward_now 가 목표 시각을 옮기기 전에 재야 한다.
	late_ns = ktime_to_ns(ktime_sub(hrtimer_cb_get_time(timer),
					hrtimer_get_expires(timer)));

	spin_lock_irqsave(&ch->lock, flags);

	if (ch->stats_skip) {
		// 첫 콜백에는 hrtimer_start 자체의 지연이 섞여 있다.
		ch->stats_skip = false;
	} else {
		// hrtimer 는 일찍 울리지 않는다. 음수가 나오면 0 으로 본다.
		u64 late = late_ns > 0 ? (u64)late_ns : 0;
		int b;

		ch->jit_count++;
		ch->jit_sum_ns += late;
		if (late > ch->jit_max_ns)
			ch->jit_max_ns = late;

		// 평균·최대만으로는 "98us 가 한 번인지 백 번인지" 를 알 수 없다.
		// 구간별 개수를 세면 분포가 보인다. 구간이 5개뿐이라 이 짧은 loop 은
		// 원자적 컨텍스트에서도 부담이 없다.
		for (b = 0; b < SERVO_JIT_BUCKETS - 1; b++) {
			if (late < servo_jit_edge_ns[b])
				break;
		}
		ch->jit_bucket[b]++;
	}

	pulse = ch->pulse_us;

	if (!pulse) {
		// 중단 요청이 들어왔다. 핀을 내리고 반복을 끝낸다.
		gpiod_set_value(ch->desc, 0);
		ch->level = false;
		ch->running = false;
		spin_unlock_irqrestore(&ch->lock, flags);
		return HRTIMER_NORESTART;	/* 다시 부르지 마 */
	}

	if (!ch->level) {
		gpiod_set_value(ch->desc, 1);	/* 펄스 시작 */
		ch->level = true;
		next_us = pulse;
	} else {
		gpiod_set_value(ch->desc, 0);	/* 펄스 끝 */
		ch->level = false;
		next_us = SERVO_PERIOD_US - pulse;
	}

	spin_unlock_irqrestore(&ch->lock, flags);

	// "지금부터" 가 아니라 "원래 울려야 했던 시각부터" 다음을 잡는다.
	// 늦게 울려도 오차가 누적되지 않는다.
	hrtimer_forward_now(timer, servo_us_to_ktime(next_us));
	return HRTIMER_RESTART;			/* 또 불러줘 */
}

// 펄스폭을 바꾼다. 멈춰 있었으면 시작시킨다.
// 이 함수는 유저의 write() 를 대신해 실행되므로 잠들어도 되는 자리다.
static void servo_set_pulse(struct servo_channel *ch, unsigned int pulse_us)
{
	unsigned long flags;
	bool need_start;

	spin_lock_irqsave(&ch->lock, flags);
	ch->pulse_us = pulse_us;
	// 이미 돌고 있으면 값만 갈아끼운다. 타이머를 건드리지 않으므로
	// 펄스가 끊기지 않는다.
	need_start = pulse_us && !ch->running;
	if (need_start) {
		ch->running = true;
		ch->level = false;	/* 첫 알람에서 HIGH 로 올라가게 */
		ch->stats_skip = true;
	}
	spin_unlock_irqrestore(&ch->lock, flags);

	// 자물쇠를 놓은 뒤에 타이머를 건다. 0 = "가능한 한 빨리".
	if (need_start)
		hrtimer_start(&ch->timer, servo_us_to_ktime(0),
			      HRTIMER_MODE_REL);
}

// ───────────────────────── 캐릭터 디바이스 ─────────────────────────

// /dev/servo0 에 write 가 들어오면 커널이 이 함수를 부른다.
//
//   buf  유저 공간 주소.  절대 직접 읽으면 안 된다.
//   len  유저가 쓰겠다고 한 바이트 수
//
// 반환값이 "내가 소비한 바이트 수" 다. 음수를 돌려주면 유저의 write() 가
// 그 오류로 실패한다.
static ssize_t servo_write(struct file *filp, const char __user *buf,
			   size_t len, loff_t *off)
{
	char kbuf[SERVO_WRITE_MAX + 1];
	char name[8];
	char arg[16];
	unsigned int pulse_us;
	struct servo_channel *ch;
	size_t n = min(len, (size_t)SERVO_WRITE_MAX);
	int ret;

	// 유저가 준 포인터는 거짓일 수도 있고, 그 사이 해제됐을 수도 있다.
	// 커널이 그대로 읽으면 죽는다. 그래서 검사까지 해 주는 이 함수로 복사한다.
	if (copy_from_user(kbuf, buf, n))
		return -EFAULT;

	kbuf[n] = '\0';	/* 유저가 보낸 건 문자열이 아니다. 끝을 직접 만든다 */
	strim(kbuf);		/* echo 가 붙이는 줄바꿈을 떼고 본다 */

	if (sscanf(kbuf, "%7s %15s", name, arg) != 2) {
		pr_warn("형식이 틀렸습니다: \"%s\" (예: pan 1500 / pan 30deg / pan off)\n",
			kbuf);
		return -EINVAL;
	}

	ch = servo_find(name);
	if (!ch) {
		pr_warn("모르는 채널: \"%s\" (pan 또는 tilt)\n", name);
		return -EINVAL;
	}

	ret = servo_parse_pulse(arg, &pulse_us);
	if (ret == -ERANGE) {
		pr_warn("%s: 범위 밖 \"%s\" (%d~%dus 또는 %d~%ddeg)\n", ch->name,
			arg, SERVO_MIN_PULSE_US, SERVO_MAX_PULSE_US,
			SERVO_MIN_DEG, SERVO_MAX_DEG);
		return ret;
	}
	if (ret) {
		pr_warn("%s: 값을 읽을 수 없습니다 \"%s\"\n", ch->name, arg);
		return ret;
	}

	servo_set_pulse(ch, pulse_us);

	if (pulse_us)
		pr_info("%s (GPIO%u) -> %uus (%ddeg)\n", ch->name, ch->gpio,
			pulse_us, servo_us_to_deg(pulse_us));
	else
		pr_info("%s (GPIO%u) -> 펄스 중단\n", ch->name, ch->gpio);

	return len;
}

// cat /dev/servo0 으로 현재 상태를 본다.
//
//   pan   1833us   30deg  on
//   tilt     -us     -deg  off
//
// 이 함수는 유저의 read() 를 대신해 실행되므로 잠들어도 되는 자리다.
// 다만 콜백과 같은 값을 읽으므로 spinlock 은 필요하다.
static ssize_t servo_read(struct file *filp, char __user *buf, size_t len,
			  loff_t *off)
{
	char out[160];
	size_t used = 0;
	int i;

	for (i = 0; i < ARRAY_SIZE(servo_ch); i++) {
		struct servo_channel *ch = &servo_ch[i];
		unsigned long flags;
		unsigned int pulse;
		bool running;

		spin_lock_irqsave(&ch->lock, flags);
		pulse = ch->pulse_us;
		running = ch->running;
		spin_unlock_irqrestore(&ch->lock, flags);

		if (running && pulse)
			used += scnprintf(out + used, sizeof(out) - used,
					  "%-4s %5uus %5ddeg  on\n",
					  ch->name, pulse,
					  servo_us_to_deg(pulse));
		else
			used += scnprintf(out + used, sizeof(out) - used,
					  "%-4s     -us      -deg  off\n",
					  ch->name);
	}

	// 오프셋 처리와 유저 공간 복사를 대신해 주는 커널 헬퍼.
	// *off 를 넘겨주므로 두 번째 read 에서는 0(EOF)이 돌아가고 cat 이 끝난다.
	return simple_read_from_buffer(buf, len, off, out, used);
}

// ───────────────────────── 버튼 인터럽트 ─────────────────────────

// 엣지가 SERVO_SETTLE_MS 동안 하나도 없었을 때만 불린다.
// 즉 지금 읽는 값은 튐이 끝난 "안정된" 값이다. 마지막 튐이 언제였는지를
// 따로 알아낼 필요가 없다 — 이 함수가 불렸다는 사실이 곧 그 증거다.
static enum hrtimer_restart servo_button_settle_fn(struct hrtimer *timer)
{
	unsigned long flags;

	if (gpiod_get_value(button_desc) == 1) {
		spin_lock_irqsave(&button_lock, flags);
		button_armed = true;		/* 떼어졌다. 다음 누름을 받는다 */
		spin_unlock_irqrestore(&button_lock, flags);
	}
	// LOW 면 아직 쥐고 있는 것이다. 다시 걸지 않는다 —
	// 뗄 때 오는 엣지가 알아서 이 타이머를 다시 건다.
	return HRTIMER_NORESTART;
}

// 하드웨어가 핀 변화를 보고 CPU 를 깨운다. 양쪽 엣지를 다 받는다 —
// 떼는 것도 봐야 "이제 놓였구나" 를 알 수 있기 때문이다.
// hrtimer 콜백과 같은 제약을 받는다: 잠들 수 없고, 짧아야 한다.
static irqreturn_t servo_button_isr(int irq, void *dev_id)
{
	// 인터럽트를 받은 시각. 이후 모든 지연 측정의 기준점이다.
	// ktime_get() 은 CLOCK_MONOTONIC 이라 유저의 clock_gettime 과 바로 비교된다.
	const ktime_t now = ktime_get();
	const int level = gpiod_get_value(button_desc);
	unsigned long flags;
	bool accepted = false;

	// 엣지가 올 때마다 정착 타이머를 뒤로 민다. 이미 걸려 있으면 취소되고
	// 새로 걸린다. 튀는 동안에는 계속 밀려서 울리지 않는다.
	hrtimer_start(&button_settle, ms_to_ktime(SERVO_SETTLE_MS),
		      HRTIMER_MODE_REL);

	spin_lock_irqsave(&button_lock, flags);
	button_irq_count++;
	if (button_armed && level == 0) {
		button_armed = false;		/* 떼어질 때까지 잠근다 */
		button_stamp = now;
		button_seq++;
		accepted = true;
	} else {
		button_reject_count++;		/* 튐이거나, 떼는 중이거나 */
	}
	spin_unlock_irqrestore(&button_lock, flags);

	// wake_up 계열은 인터럽트 문맥에서 불러도 된다 (잠들지 않는다).
	// 지연을 줄이려면 알림은 첫 엣지에 곧바로 해야 한다. 안정될 때까지
	// 기다렸다 알리면 지연이 20ms 가 되어 인터럽트를 쓴 의미가 사라진다.
	if (accepted)
		wake_up_interruptible(&button_wq);

	return IRQ_HANDLED;
}

static int servo_button_setup(void)
{
	int ret;

	// IRQ 를 걸기 전에 타이머를 준비해 둔다. 첫 인터럽트가 곧바로 이걸 쓴다.
	hrtimer_init(&button_settle, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	button_settle.function = servo_button_settle_fn;
	button_armed = true;

	ret = gpio_request_one(SERVO_BUTTON_GPIO, GPIOF_IN, "servo-button");
	if (ret) {
		pr_err("GPIO%d 요청 실패 (%d)\n", SERVO_BUTTON_GPIO, ret);
		return ret;
	}

	button_desc = gpio_to_desc(SERVO_BUTTON_GPIO);
	if (!button_desc) {
		pr_err("GPIO%d 손잡이를 얻지 못했습니다\n", SERVO_BUTTON_GPIO);
		ret = -ENODEV;
		goto err_free;
	}

	// 풀업을 켠다. 이 핀의 기본값은 풀다운이라(라즈베리파이는 GPIO 9~27 이 기본
	// 풀다운) 켜지 않으면 버튼을 안 눌러도 계속 LOW 로 읽힌다.
	// 전원이 나가면 초기화되므로 적재할 때마다 켜야 한다.
	ret = gpiod_set_config(button_desc,
			       PIN_CONF_PACKED(PIN_CONFIG_BIAS_PULL_UP, 0));
	if (ret)
		pr_warn("GPIO%d 풀업 설정 실패 (%d). 버튼이 오작동할 수 있습니다\n",
			SERVO_BUTTON_GPIO, ret);

	button_irq = gpiod_to_irq(button_desc);
	if (button_irq < 0) {
		pr_err("GPIO%d 인터럽트 번호를 얻지 못했습니다 (%d)\n",
		       SERVO_BUTTON_GPIO, button_irq);
		ret = button_irq;
		goto err_free;
	}

	// 양쪽 엣지를 다 받는다. 누름만 받으면, 깔끔하게 떼었을 때 엣지가 하나도
	// 안 와서 잠금이 영영 안 풀린다.
	// IRQF_TRIGGER_BOTH 라는 상수는 없다. 둘을 OR 로 묶는 게 양쪽 엣지다.
	ret = request_irq(button_irq, servo_button_isr,
			  IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			  "servo-button", NULL);
	if (ret) {
		pr_err("IRQ %d 등록 실패 (%d)\n", button_irq, ret);
		goto err_free;
	}

	pr_info("버튼 GPIO%d 확보, IRQ %d, 양쪽 엣지, 정착 대기 %dms\n",
		SERVO_BUTTON_GPIO, button_irq, SERVO_SETTLE_MS);
	return 0;

err_free:
	button_irq = -1;
	button_desc = NULL;
	gpio_free(SERVO_BUTTON_GPIO);
	return ret;
}

static void servo_button_teardown(void)
{
	if (button_irq >= 0) {
		free_irq(button_irq, NULL);	/* 실행 중인 핸들러가 끝날 때까지 기다린다 */
		button_irq = -1;
	}
	// IRQ 를 뗀 뒤에 세운다 — 반대로 하면 핸들러가 타이머를 다시 걸 수 있다.
	hrtimer_cancel(&button_settle);
	if (button_desc) {
		gpio_free(SERVO_BUTTON_GPIO);
		button_desc = NULL;
	}
}

// /dev/button0 을 read 하면 새 눌림이 있을 때까지 잠든다.
//
// 폴링과 다른 점: 여기서 잠든 프로세스는 CPU 를 전혀 쓰지 않는다. 스케줄러가
// 아예 실행 대상에서 빼놓기 때문이다. 인터럽트가 와야 다시 깨어난다.
static ssize_t servo_button_read(struct file *filp, char __user *buf, size_t len,
				 loff_t *off)
{
	struct button_reader *r = filp->private_data;
	unsigned long flags;
	char out[96];
	size_t used;
	u64 seq;
	ktime_t stamp;
	int ret;

	if (!r)
		return -EINVAL;

	// 조건이 이미 참이면 안 자고 바로 지나간다.
	// 0 이 아닌 값을 돌려주면 신호에 깨진 것이다 (-ERESTARTSYS).
	ret = wait_event_interruptible(button_wq, READ_ONCE(button_seq) != r->seen);
	if (ret)
		return ret;

	spin_lock_irqsave(&button_lock, flags);
	seq = button_seq;
	stamp = button_stamp;
	spin_unlock_irqrestore(&button_lock, flags);
	r->seen = seq;

	used = scnprintf(out, sizeof(out), "press %llu ktime=%lld\n", seq,
			 ktime_to_ns(stamp));
	if (len < used)
		return -EINVAL;
	if (copy_to_user(buf, out, used))
		return -EFAULT;

	return used;
}

// ───────────────────────── 지터 통계 (sysfs) ─────────────────────────
//
// /dev 는 데이터가 흐르는 통로고, /sys 는 값을 하나씩 보여 주는 곳이다.
// 통계처럼 "가끔 들여다보는 숫자" 는 sysfs 가 제자리다.
//
//   cat   /sys/class/servo/servo0/jitter    읽기
//   echo 0 > /sys/class/servo/servo0/jitter  초기화 (root)
//
// 한계: 재는 것은 "콜백이 예정대로 불렸는가" 이지, 핀 전압이 실제로 뒤집힌
// 시각이 아니다. 콜백 안에서 gpiod_set_value 까지 걸리는 시간은 여기 안 들어
// 있다. 진짜 파형을 보려면 오실로스코프가 필요하다.
static ssize_t jitter_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	size_t used = 0;
	int i;

	for (i = 0; i < ARRAY_SIZE(servo_ch); i++) {
		struct servo_channel *ch = &servo_ch[i];
		unsigned long flags;
		u64 count, sum, max, avg;

		u64 bucket[SERVO_JIT_BUCKETS];
		int b;

		spin_lock_irqsave(&ch->lock, flags);
		count = ch->jit_count;
		sum = ch->jit_sum_ns;
		max = ch->jit_max_ns;
		memcpy(bucket, ch->jit_bucket, sizeof(bucket));
		spin_unlock_irqrestore(&ch->lock, flags);

		avg = count ? sum / count : 0;

		// 커널에는 부동소수점이 없다. 정수 나눗셈과 나머지로 소수점을 만든다.
		used += scnprintf(buf + used, PAGE_SIZE - used,
				  "%-4s 표본 %llu  평균 %llu.%01lluus  최대 %llu.%01lluus\n",
				  ch->name, count,
				  avg / 1000, (avg % 1000) / 100,
				  max / 1000, (max % 1000) / 100);

		for (b = 0; b < SERVO_JIT_BUCKETS; b++) {
			// 비율도 정수로 만든다. 1000 배 한 뒤 소수점을 끼워 넣는다.
			u64 permil = count ? bucket[b] * 1000 / count : 0;

			used += scnprintf(buf + used, PAGE_SIZE - used,
					  "     %-16s %8llu  %3llu.%01llu%%\n",
					  servo_jit_label[b], bucket[b],
					  permil / 10, permil % 10);
		}
	}

	used += scnprintf(buf + used, PAGE_SIZE - used,
			  "주기 %dus, 1도 = %dus\n", SERVO_PERIOD_US,
			  (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US) /
			  (SERVO_MAX_DEG - SERVO_MIN_DEG));
	return used;
}

// 무엇을 쓰든 초기화한다. 측정을 여러 번 나눠 하려면 매번 0 부터 시작해야 한다.
static ssize_t jitter_store(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t count)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(servo_ch); i++) {
		struct servo_channel *ch = &servo_ch[i];
		unsigned long flags;

		spin_lock_irqsave(&ch->lock, flags);
		ch->jit_count = 0;
		ch->jit_sum_ns = 0;
		ch->jit_max_ns = 0;
		memset(ch->jit_bucket, 0, sizeof(ch->jit_bucket));
		spin_unlock_irqrestore(&ch->lock, flags);
	}
	pr_info("지터 통계 초기화\n");
	return count;
}

// _RW 는 읽기·쓰기 속성(0644)을 만든다. 이름이 그대로 파일 이름이 되고,
// jitter_show / jitter_store 를 찾아 연결한다.
static DEVICE_ATTR_RW(jitter);

// 버튼 계측 — 채터링이 얼마나 걸러졌는지 숫자로 보여 준다.
//
//   cat /sys/class/servo/servo0/button
//   irq 47  accept 12  reject 35  armed 1  level 1
//
// irq 는 하드웨어가 실제로 올린 인터럽트 수, accept 는 그중 누름으로 인정한 수,
// reject 는 튐이거나 떼는 중이라 버린 수다. 폴링에서는 이 차이가 애초에 보이지
// 않는다 — 샘플링이라 짧은 튐을 놓치기 때문이다.
static ssize_t button_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	unsigned long flags;
	u64 irqs, accepts, rejects;
	bool armed;
	int level;

	level = button_desc ? gpiod_get_value(button_desc) : -1;

	spin_lock_irqsave(&button_lock, flags);
	irqs = button_irq_count;
	accepts = button_seq;
	rejects = button_reject_count;
	armed = button_armed;
	spin_unlock_irqrestore(&button_lock, flags);

	return scnprintf(buf, PAGE_SIZE,
			 "irq %llu  accept %llu  reject %llu  armed %d  level %d\n"
			 "GPIO%d, 양쪽 엣지, 정착 대기 %dms\n",
			 irqs, accepts, rejects, armed ? 1 : 0, level,
			 SERVO_BUTTON_GPIO, SERVO_SETTLE_MS);
}
static DEVICE_ATTR_RO(button);

// ───────────────────────── 장치 두 개 가르기 ─────────────────────────
//
// major 는 "어느 드라이버냐", minor 는 "그 드라이버의 몇 번째 장치냐" 다.
// 같은 함수 표를 쓰되 minor 로 갈라 보낸다.
//
//   minor 0 → /dev/servo0    펄스 지시 · 상태 읽기
//   minor 1 → /dev/button0   눌릴 때까지 대기

static int servo_fops_open(struct inode *inode, struct file *filp)
{
	struct button_reader *r;
	unsigned long flags;

	if (iminor(inode) != SERVO_MINOR_BUTTON)
		return 0;

	// 연 쪽마다 "어디까지 봤는지" 를 따로 들고 있어야, 두 프로그램이 동시에
	// 열어도 서로의 이벤트를 훔치지 않는다.
	r = kzalloc(sizeof(*r), GFP_KERNEL);
	if (!r)
		return -ENOMEM;

	spin_lock_irqsave(&button_lock, flags);
	r->seen = button_seq;		/* 연 시점 이전의 눌림은 못 본 것으로 */
	spin_unlock_irqrestore(&button_lock, flags);

	filp->private_data = r;
	return 0;
}

static int servo_fops_release(struct inode *inode, struct file *filp)
{
	if (iminor(inode) == SERVO_MINOR_BUTTON)
		kfree(filp->private_data);
	return 0;
}

static ssize_t servo_fops_read(struct file *filp, char __user *buf, size_t len,
			       loff_t *off)
{
	if (iminor(file_inode(filp)) == SERVO_MINOR_BUTTON)
		return servo_button_read(filp, buf, len, off);
	return servo_read(filp, buf, len, off);
}

static ssize_t servo_fops_write(struct file *filp, const char __user *buf,
				size_t len, loff_t *off)
{
	if (iminor(file_inode(filp)) != SERVO_MINOR_SERVO)
		return -EINVAL;		/* button0 에는 쓸 것이 없다 */
	return servo_write(filp, buf, len, off);
}

// .owner 는 참조 카운트용이다. 누가 장치를 열어 두면 커널이 이 값을 보고
// rmmod 를 막아 준다 — 쓰는 중인 드라이버가 사라지면 커널이 죽으니까.
static const struct file_operations servo_fops = {
	.owner   = THIS_MODULE,
	.open    = servo_fops_open,
	.release = servo_fops_release,
	.write   = servo_fops_write,
	.read    = servo_fops_read,
};

// ───────────────────────── 적재 / 제거 ─────────────────────────

// 핀을 커널에 요청하고 출력 방향으로 세운 뒤, 알람시계를 준비한다.
// GPIOF_OUT_INIT_LOW = "출력으로, 처음엔 LOW 로".
// 처음부터 LOW 인 게 중요하다 — HIGH 로 시작하면 서보에 엉뚱한 신호가 간다.
static int servo_channel_setup(struct servo_channel *ch)
{
	int ret;

	ret = gpio_request_one(ch->gpio, GPIOF_OUT_INIT_LOW, ch->label);
	if (ret) {
		// -EBUSY(-16) 면 다른 드라이버가 이미 쓰고 있다는 뜻이다.
		pr_err("GPIO%u 요청 실패 (%d)\n", ch->gpio, ret);
		return ret;
	}

	// 번호 대신 들고 다닐 손잡이를 받아 둔다.
	ch->desc = gpio_to_desc(ch->gpio);
	if (!ch->desc) {
		pr_err("GPIO%u 손잡이를 얻지 못했습니다\n", ch->gpio);
		gpio_free(ch->gpio);
		return -ENODEV;
	}

	spin_lock_init(&ch->lock);
	ch->pulse_us = 0;
	ch->level = false;
	ch->running = false;

	// CLOCK_MONOTONIC: 시스템 시간이 바뀌어도 흔들리지 않는 시계.
	// 사용자가 날짜를 고쳐도 펄스가 튀면 안 된다.
	// MODE_REL: "지금부터 얼마 뒤" 로 예약하겠다는 뜻.
	hrtimer_init(&ch->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ch->timer.function = servo_tick;	/* 울리면 이걸 불러라 */

	pr_info("GPIO%u 확보 (%s)\n", ch->gpio, ch->label);
	return 0;
}

// 타이머를 세우고 핀을 반납한다.
//
// hrtimer_cancel 이 이 모듈에서 가장 중요한 한 줄이다.
//   ① 앞으로 안 울리게 예약을 취소하고
//   ② 다른 CPU 에서 콜백이 돌고 있으면 끝날 때까지 기다린다
// 기다리지 않고 모듈을 지우면, 사라진 코드를 실행 중인 CPU 가 커널을 죽인다.
static void servo_channel_teardown(struct servo_channel *ch)
{
	if (!ch->desc)
		return;

	hrtimer_cancel(&ch->timer);
	gpiod_set_value(ch->desc, 0);	/* 나가기 전에 LOW 로 내려 둔다 */
	gpio_free(ch->gpio);
	ch->desc = NULL;
}

// 커널 코드의 정리 방식: 성공한 것만 거꾸로 되돌린다.
// 유저 프로그램은 죽으면 OS 가 회수해 주지만, 커널은 회수해 줄 상위 존재가 없다.
static int __init servo_init(void)
{
	int ret, i;

	// (1) 핀과 타이머를 먼저 준비한다. 없으면 /dev/servo0 을 만들 이유가 없다.
	for (i = 0; i < ARRAY_SIZE(servo_ch); i++) {
		ret = servo_channel_setup(&servo_ch[i]);
		if (ret)
			goto err_channel;
	}

	// (2) 버튼을 확보하고 인터럽트를 건다.
	ret = servo_button_setup();
	if (ret)
		goto err_channel;

	// (3) major/minor 번호를 받는다. 0 = "빈 번호 알아서 주세요".
	//     이번엔 minor 를 2개 받는다 — servo0 과 button0.
	ret = alloc_chrdev_region(&servo_devno, 0, SERVO_MINOR_COUNT,
				  SERVO_CLASS_NAME);
	if (ret) {
		pr_err("장치 번호를 받지 못했습니다 (%d)\n", ret);
		goto err_button;
	}

	// (4) 받은 번호와 함수 표를 묶어 커널에 등록한다.
	cdev_init(&servo_cdev, &servo_fops);
	servo_cdev.owner = THIS_MODULE;
	ret = cdev_add(&servo_cdev, servo_devno, SERVO_MINOR_COUNT);
	if (ret) {
		pr_err("cdev 등록 실패 (%d)\n", ret);
		goto err_region;
	}

	// (5) 번호에 이름을 붙이기 위한 분류를 만든다 -> /sys/class/servo
	servo_class = class_create(THIS_MODULE, SERVO_CLASS_NAME);
	if (IS_ERR(servo_class)) {
		ret = PTR_ERR(servo_class);
		pr_err("class 생성 실패 (%d)\n", ret);
		goto err_cdev;
	}

	// (6) /dev/servo0 을 만들어 달라고 요청한다.
	servo_device = device_create(servo_class, NULL,
				     MKDEV(MAJOR(servo_devno), SERVO_MINOR_SERVO),
				     NULL, SERVO_DEV_NAME);
	if (IS_ERR(servo_device)) {
		ret = PTR_ERR(servo_device);
		pr_err("device 생성 실패 (%d)\n", ret);
		goto err_class;
	}

	// (7) /dev/button0 — 같은 드라이버의 두 번째 장치.
	button_device = device_create(servo_class, NULL,
				      MKDEV(MAJOR(servo_devno), SERVO_MINOR_BUTTON),
				      NULL, SERVO_BUTTON_NAME);
	if (IS_ERR(button_device)) {
		ret = PTR_ERR(button_device);
		pr_err("button device 생성 실패 (%d)\n", ret);
		goto err_servo_dev;
	}

	// (8) 통계를 읽을 sysfs 파일을 단다. 없어도 서보는 동작하므로 실패해도
	//     경고만 찍고 계속한다.
	if (device_create_file(servo_device, &dev_attr_jitter))
		pr_warn("jitter 속성을 만들지 못했습니다 (동작에는 지장 없음)\n");
	if (device_create_file(servo_device, &dev_attr_button))
		pr_warn("button 속성을 만들지 못했습니다 (동작에는 지장 없음)\n");

	pr_info("적재됨 — major=%d, /dev/%s(minor %d) /dev/%s(minor %d), "
		"서보 GPIO%u/%u, 버튼 GPIO%u\n",
		MAJOR(servo_devno), SERVO_DEV_NAME, SERVO_MINOR_SERVO,
		SERVO_BUTTON_NAME, SERVO_MINOR_BUTTON,
		SERVO_PAN_GPIO, SERVO_TILT_GPIO, SERVO_BUTTON_GPIO);
	return 0;

err_servo_dev:
	device_destroy(servo_class, MKDEV(MAJOR(servo_devno), SERVO_MINOR_SERVO));
err_class:
	class_destroy(servo_class);
err_cdev:
	cdev_del(&servo_cdev);
err_region:
	unregister_chrdev_region(servo_devno, SERVO_MINOR_COUNT);
err_button:
	servo_button_teardown();
err_channel:
	// i 번째에서 실패했으므로 0 .. i-1 까지만 돌려준다.
	while (--i >= 0)
		servo_channel_teardown(&servo_ch[i]);
	return ret;
}

// 순서가 중요하다.
//   먼저 장치 노드를 없앤다    -> 새 요청이 들어올 길을 끊는다
//   그다음 인터럽트를 뗀다      -> 더 이상 핸들러가 불리지 않게 한다
//   그다음 타이머를 세운다      -> 더 이상 콜백이 불리지 않게 한다
//   마지막에 핀을 반납한다      -> 아무도 안 만지는 상태에서
static void __exit servo_exit(void)
{
	int i;

	device_remove_file(servo_device, &dev_attr_button);
	device_remove_file(servo_device, &dev_attr_jitter);
	device_destroy(servo_class, MKDEV(MAJOR(servo_devno), SERVO_MINOR_BUTTON));
	device_destroy(servo_class, MKDEV(MAJOR(servo_devno), SERVO_MINOR_SERVO));
	class_destroy(servo_class);
	cdev_del(&servo_cdev);
	unregister_chrdev_region(servo_devno, SERVO_MINOR_COUNT);

	// free_irq 는 다른 CPU 에서 핸들러가 돌고 있으면 끝날 때까지 기다린다.
	// hrtimer_cancel 과 같은 이유로 반드시 필요하다.
	servo_button_teardown();

	for (i = ARRAY_SIZE(servo_ch) - 1; i >= 0; i--)
		servo_channel_teardown(&servo_ch[i]);

	pr_info("제거됨\n");
}

module_init(servo_init);
module_exit(servo_exit);

// MODULE_LICENSE 가 없거나 GPL 호환이 아니면 커널이 taint 경고를 찍고
// GPL 전용으로 표시된 커널 함수를 쓸 수 없게 된다.
MODULE_LICENSE("GPL");
MODULE_AUTHOR("JSH");
MODULE_DESCRIPTION("MG90 pan/tilt servo PWM driver with emergency stop button");
MODULE_VERSION("0.9");
