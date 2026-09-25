#include "config.hpp"

#include <fstream>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>

#include <nlohmann/json.hpp>

using json = nlohmann::json;

namespace {

// JSON 객체 한 덩어리에서 "아는 키만" 읽어가고, 끝에 done() 으로 남은 키를
// 검사한다. 오타 난 키를 조용히 무시하지 않으려는 장치다.
class Section {
public:
    Section(const json& j, std::string where) : j_(j), where_(std::move(where)) {
        if (!j_.is_object()) {
            throw std::runtime_error(where_ + ": 객체(...{ }...)여야 합니다");
        }
    }

    template <class T>
    Section& get(const char* key, T& out) {
        seen_.insert(key);
        auto it = j_.find(key);
        if (it == j_.end()) return *this;  // 없으면 기본값 유지
        try {
            out = it->get<T>();
        } catch (const json::exception& e) {
            throw std::runtime_error(std::string(where_) + "." + key + ": 값의 형식이 맞지 않습니다");
        }
        return *this;
    }

    // 중첩 객체. 없으면 nullptr.
    const json* sub(const char* key) {
        seen_.insert(key);
        auto it = j_.find(key);
        return it == j_.end() ? nullptr : &*it;
    }

    void done() const {
        for (const auto& item : j_.items()) {
            if (seen_.count(item.key()) == 0) {
                throw std::runtime_error(where_ + ": 알 수 없는 설정 키 '" + item.key() + "'");
            }
        }
    }

private:
    const json& j_;
    std::string where_;
    std::set<std::string> seen_;
};

void loadPid(const json& j, const std::string& where, PIDConfig& pid) {
    Section s(j, where);
    s.get("kp", pid.kp)
     .get("ki", pid.ki)
     .get("kd", pid.kd)
     .get("max_step", pid.max_step)
     .get("integral_limit", pid.integral_limit);
    s.done();
}

json dumpPid(const PIDConfig& p) {
    return json{{"kp", p.kp}, {"ki", p.ki}, {"kd", p.kd},
                {"max_step", p.max_step}, {"integral_limit", p.integral_limit}};
}

}  // namespace

Config Config::load(const std::string& path) {
    Config c;
    if (path.empty()) return c;

    std::ifstream in(path);
    if (!in) return c;  // 설정 파일이 없으면 기본값 그대로

    json root;
    try {
        in >> root;
    } catch (const json::parse_error& e) {
        throw std::runtime_error(path + ": JSON 문법 오류 — " + e.what());
    }

    Section top(root, path);

    if (const json* j = top.sub("camera")) {
        Section s(*j, "camera");
        s.get("width", c.camera.width)
         .get("height", c.camera.height)
         .get("fps", c.camera.fps)
         .get("bitrate", c.camera.bitrate)
         .get("rotation", c.camera.rotation)
         .get("hflip", c.camera.hflip)
         .get("vflip", c.camera.vflip);
        s.done();
    }
    if (const json* j = top.sub("stream")) {
        Section s(*j, "stream");
        s.get("enabled", c.stream.enabled)
         .get("rtsp_url", c.stream.rtsp_url)
         .get("rtsp_transport", c.stream.rtsp_transport)
         .get("ffmpeg_loglevel", c.stream.ffmpeg_loglevel);
        s.done();
    }
    if (const json* j = top.sub("detect")) {
        Section s(*j, "detect");
        s.get("width", c.detect.width)
         .get("height", c.detect.height)
         .get("fps", c.detect.fps)
         .get("backend", c.detect.backend)
         .get("confidence", c.detect.confidence)
         .get("model_dir", c.detect.model_dir)
         .get("interval", c.detect.interval)
         .get("tracker", c.detect.tracker)
         .get("redetect_interval", c.detect.redetect_interval);
        s.done();
    }
    if (const json* j = top.sub("motor")) {
        Section s(*j, "motor");
        s.get("backend", c.motor.backend)
         .get("pan_pin", c.motor.pan_pin)
         .get("tilt_pin", c.motor.tilt_pin)
         .get("min_pulse_ms", c.motor.min_pulse_ms)
         .get("max_pulse_ms", c.motor.max_pulse_ms)
         .get("pan_min", c.motor.pan_min)
         .get("pan_max", c.motor.pan_max)
         .get("tilt_min", c.motor.tilt_min)
         .get("tilt_max", c.motor.tilt_max)
         .get("pan_home", c.motor.pan_home)
         .get("tilt_home", c.motor.tilt_home)
         .get("pan_invert", c.motor.pan_invert)
         .get("tilt_invert", c.motor.tilt_invert)
         .get("idle_detach", c.motor.idle_detach)
         .get("pigpio_host", c.motor.pigpio_host)
         .get("pigpio_port", c.motor.pigpio_port)
         .get("kservo_path", c.motor.kservo_path);
        s.done();
    }
    if (const json* j = top.sub("track")) {
        Section s(*j, "track");
        s.get("enabled", c.track.enabled)
         .get("deadzone", c.track.deadzone)
         .get("lost_timeout", c.track.lost_timeout)
         .get("recenter_on_lost", c.track.recenter_on_lost)
         .get("compensate_latency", c.track.compensate_latency)
         .get("latency_ms", c.track.latency_ms)
         .get("pan_deg_to_err", c.track.pan_deg_to_err)
         .get("tilt_deg_to_err", c.track.tilt_deg_to_err);
        if (const json* p = s.sub("pan_pid")) loadPid(*p, "track.pan_pid", c.track.pan_pid);
        if (const json* p = s.sub("tilt_pid")) loadPid(*p, "track.tilt_pid", c.track.tilt_pid);
        s.done();
    }
    if (const json* j = top.sub("debug")) {
        Section s(*j, "debug");
        s.get("log_level", c.debug.log_level)
         .get("snapshot_path", c.debug.snapshot_path)
         .get("snapshot_interval", c.debug.snapshot_interval);
        s.done();
    }
    top.done();
    return c;
}

std::string Config::dump() const {
    json j;
    j["camera"] = {{"width", camera.width},
                   {"height", camera.height}, {"fps", camera.fps},
                   {"bitrate", camera.bitrate}, {"rotation", camera.rotation},
                   {"hflip", camera.hflip}, {"vflip", camera.vflip}};
    j["stream"] = {{"enabled", stream.enabled}, {"rtsp_url", stream.rtsp_url},
                   {"rtsp_transport", stream.rtsp_transport},
                   {"ffmpeg_loglevel", stream.ffmpeg_loglevel}};
    j["detect"] = {{"width", detect.width}, {"height", detect.height},
                   {"fps", detect.fps}, {"backend", detect.backend},
                   {"confidence", detect.confidence}, {"model_dir", detect.model_dir},
                   {"interval", detect.interval}, {"tracker", detect.tracker},
                   {"redetect_interval", detect.redetect_interval}};
    j["motor"] = {{"backend", motor.backend}, {"pan_pin", motor.pan_pin},
                  {"tilt_pin", motor.tilt_pin}, {"min_pulse_ms", motor.min_pulse_ms},
                  {"max_pulse_ms", motor.max_pulse_ms}, {"pan_min", motor.pan_min},
                  {"pan_max", motor.pan_max}, {"tilt_min", motor.tilt_min},
                  {"tilt_max", motor.tilt_max}, {"pan_home", motor.pan_home},
                  {"tilt_home", motor.tilt_home}, {"pan_invert", motor.pan_invert},
                  {"tilt_invert", motor.tilt_invert}, {"idle_detach", motor.idle_detach},
                  {"pigpio_host", motor.pigpio_host}, {"pigpio_port", motor.pigpio_port},
                  {"kservo_path", motor.kservo_path}};
    j["track"] = {{"enabled", track.enabled}, {"deadzone", track.deadzone},
                  {"lost_timeout", track.lost_timeout},
                  {"recenter_on_lost", track.recenter_on_lost},
                  {"compensate_latency", track.compensate_latency},
                  {"latency_ms", track.latency_ms},
                  {"pan_deg_to_err", track.pan_deg_to_err},
                  {"tilt_deg_to_err", track.tilt_deg_to_err},
                  {"pan_pid", dumpPid(track.pan_pid)},
                  {"tilt_pid", dumpPid(track.tilt_pid)}};
    j["debug"] = {{"log_level", debug.log_level}, {"snapshot_path", debug.snapshot_path},
                  {"snapshot_interval", debug.snapshot_interval}};
    return j.dump(2);
}
