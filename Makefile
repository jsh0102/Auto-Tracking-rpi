TARGET   := camtracker
BUILDDIR := build

SRCS := $(wildcard src/*.cpp)
OBJS := $(SRCS:%.cpp=$(BUILDDIR)/%.o)
DEPS := $(OBJS:.o=.d)

PKGS := opencv4 libcamera

CXX      ?= g++
# -MMD -MP: 헤더 의존성(.d) 자동 생성 — .hpp 를 고치면 그걸 쓰는 .o 가 전부 재빌드된다
CXXFLAGS += -std=c++17 -Wall -Wextra -O2 -MMD -MP -Isrc -Ithird_party
CXXFLAGS += $(shell pkg-config --cflags $(PKGS))

# opencv4 는 pkg-config 가 전체 모듈을 뱉어낸다. 실제로 쓰는 건 core/imgproc/
# imgcodecs/dnn 넷뿐이라 직접 적어 링크 시간과 의존성을 줄인다.
LDLIBS := -lopencv_core -lopencv_imgproc -lopencv_imgcodecs -lopencv_dnn
LDLIBS += -lopencv_tracking -lopencv_video
LDLIBS += -lcamera -lcamera-base
LDLIBS += -lpigpiod_if2 -lpthread
LDFLAGS += $(shell pkg-config --libs-only-L $(PKGS))

MEDIAMTX := ./bin/mediamtx
ARGS     ?=

all: $(BUILDDIR)/$(TARGET)

$(BUILDDIR)/$(TARGET): $(OBJS)
	$(CXX) $(LDFLAGS) $^ -o $@ $(LDLIBS)
	@echo "빌드 완료 -> $@"

$(BUILDDIR)/%.o: %.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# MediaMTX(RTSP 서버)를 같이 띄우고 종료 시 같이 내린다.
# 이미 8554 를 쓰고 있으면 그걸 그대로 쓴다.
#   make run
#   make run ARGS="--no-track"
run: all
	@if ss -ltn 2>/dev/null | grep -q ':8554 '; then \
	    echo "==> 8554 포트에 RTSP 서버가 이미 떠 있습니다. 그대로 사용합니다."; \
	    $(BUILDDIR)/$(TARGET) $(ARGS); \
	elif [ -x $(MEDIAMTX) ]; then \
	    echo "==> MediaMTX 시작"; \
	    $(MEDIAMTX) mediamtx.yml & \
	    MTX=$$!; \
	    trap "kill $$MTX 2>/dev/null" EXIT INT TERM; \
	    sleep 1; \
	    echo "==> 스트림: rtsp://$$(hostname -I | awk '{print $$1}'):8554/cam"; \
	    $(BUILDDIR)/$(TARGET) $(ARGS); \
	else \
	    echo "MediaMTX 가 없습니다($(MEDIAMTX)). scripts/install.sh 를 실행하세요." >&2; \
	    exit 1; \
	fi

# 개발용 도구. 본 프로그램과 별개로 빌드한다(main 이 각자 있으므로).
tools: tools/servo_test tools/button_poll tools/button_wait

tools/servo_test: tools/servo_test.cpp src/config.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@ -lpigpiod_if2
	@echo "빌드 완료 -> $@"

# 유저스페이스 폴링 기준선 (A 단계). pigpio 를 쓰지 않는다 — 비교 대상이므로.
tools/button_poll: tools/button_poll.cpp
	$(CXX) -std=c++17 -Wall -Wextra -O2 $< -o $@
	@echo "빌드 완료 -> $@"

# 커널 인터럽트를 기다리는 쪽 (B 단계)
tools/button_wait: tools/button_wait.cpp
	$(CXX) -std=c++17 -Wall -Wextra -O2 $< -o $@
	@echo "빌드 완료 -> $@"

clean:
	rm -rf $(BUILDDIR) tools/servo_test tools/button_poll tools/button_wait

-include $(DEPS)

.PHONY: all run clean tools
