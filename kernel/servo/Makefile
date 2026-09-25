# 커널 모듈 빌드는 커널 쪽 Makefile 에 일을 넘기는 방식이다(kbuild).
#   obj-m  : 모듈로 만들 오브젝트. servo.o -> servo.ko
#   -C KDIR: 커널 빌드 시스템 디렉터리로 이동해서 make 를 돌린다
#   M=     : "모듈 소스는 여기 있다" 고 알려준다
#
# M= 에 $(PWD) 를 쓰면 안 된다. make -C 로 디렉터리를 옮겨도 PWD 는
# 처음 make 를 부른 곳을 가리켜서, 엉뚱한 디렉터리를 빌드한다.
# $(CURDIR) 는 이 Makefile 이 있는 곳을 가리킨다.

obj-m := servo.o

KDIR  ?= /lib/modules/$(shell uname -r)/build
MODULE := servo

all:
	$(MAKE) -C $(KDIR) M=$(CURDIR) modules

clean:
	$(MAKE) -C $(KDIR) M=$(CURDIR) clean

# 편의 타겟. 적재 결과는 dmesg 에 찍힌다.
load: all
	sudo insmod $(MODULE).ko
	@dmesg | tail -3

unload:
	sudo rmmod $(MODULE)
	@dmesg | tail -3

reload: unload load

info: all
	modinfo $(MODULE).ko

.PHONY: all clean load unload reload info
