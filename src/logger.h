// (c) 2017 Blai Bonet

#ifndef LOGGER_H
#define LOGGER_H

#include <cassert>
#include <iostream>
#include <string>
#include "utils.h"

// Inspired by logger in Arcade Learning Environment
namespace logging{
class Logger {
  public:
    enum mode_t {
      Debug = 0,
      Info = 1,
      Warning = 2,
      Error = 3,
      Stats = 4,
      Silent = 5
    };

    enum color_t {
      color_normal = 0,
      color_red = 1,
      color_green = 2,
      color_yellow = 3,
      color_blue = 4,
      color_magenta = 5,
      color_cyan = 6
    };

    struct Mode {
        mode_t mode_;
        int debug_severity_;
        bool continuation_;
        Mode(mode_t mode, int debug_severity = 0, bool continuation = false)
          : mode_(mode),
            debug_severity_(debug_severity),
            continuation_(continuation) {
        }
    };

    struct Continuation : public Mode {
        Continuation(const Mode &mode)
          : Mode(mode.mode_, mode.debug_severity_, true) {
        }
        Continuation(mode_t mode, int debug_severity)
          : Mode(mode, debug_severity, true) {
        }
    };

    struct DebugMode : public Mode {
        DebugMode(int severity, bool continuation = false)
          : Mode(logging::Logger::Debug, severity, continuation) {
        }
    };

  public:
    Logger() { }
    ~Logger() { }

    static void set_output_stream(std::ostream &output_stream) {
        logging::Logger::output_stream_ = &output_stream;
    }
    static void set_mode(mode_t mode) {
        logging::Logger::current_mode_ = mode;
    }
    static int current_mode() {
        return logging::Logger::current_mode_;
    }
    static void set_debug_threshold(int debug_threshold) {
        logging::Logger::current_debug_threshold_ = debug_threshold;
    }
    static int current_debug_threshold() {
        return logging::Logger::current_debug_threshold_;
    }
    static void set_use_color(bool use_color) {
        logging::Logger::use_color_ = use_color;
    }
    static bool use_color() {
        return Logger::use_color_;
    }

    static bool available() {
        return logging::Logger::output_stream_ != nullptr;
    }
    static std::ostream& output_stream() {
        return *logging::Logger::output_stream_;
    }

    static std::string prefix(mode_t log) {
        std::string str;
        if( log == Debug )
            str += logging::Logger::blue() + "logging::Logger::Debug: ";
        else if( log == Info )
            str += logging::Logger::green() + "logging::Logger::Info: ";
        else if( log == Warning )
            str += logging::Logger::magenta() + "logging::Logger::Warning: ";
        else if( log == Error )
            str += logging::Logger::red() + "logging::Logger::Error: ";
        else if( log == Stats )
            str += logging::Logger::yellow() + "logging::Logger::Stats: ";
        return str + logging::Logger::normal();
    }

    static std::string color(color_t color) {
        if( logging::Logger::use_color_ ) {
            if( color == logging::Logger::color_normal )
                return Utils::normal();
            else if( color == logging::Logger::color_red )
                return Utils::red();
            else if( color == logging::Logger::color_green )
                return Utils::green();
            else if( color == logging::Logger::color_yellow )
                return Utils::yellow();
            else if( color == logging::Logger::color_blue )
                return Utils::blue();
            else if( color == logging::Logger::color_magenta )
                return Utils::magenta();
            else if( color == logging::Logger::color_cyan )
                return Utils::cyan();
        }
        return "";
    }
    static std::string normal() {
        return logging::Logger::color(logging::Logger::color_normal);
    }
    static std::string red() {
        return logging::Logger::color(logging::Logger::color_red);
    }
    static std::string green() {
        return logging::Logger::color(logging::Logger::color_green);
    }
    static std::string yellow() {
        return logging::Logger::color(logging::Logger::color_yellow);
    }
    static std::string blue() {
        return logging::Logger::color(logging::Logger::color_blue);
    }
    static std::string magenta() {
        return logging::Logger::color(logging::Logger::color_magenta);
    }
    static std::string cyan() {
        return logging::Logger::color(logging::Logger::color_cyan);
    }

  protected:
    static std::ostream *output_stream_;
    static mode_t current_mode_;
    static int current_debug_threshold_;
    static bool use_color_;
};
}
template<typename T>
inline logging::Logger::Mode operator<<(logging::Logger::mode_t log, const T &value) {
    if( logging::Logger::available() && (log >= logging::Logger::current_mode()) )
        logging::Logger::output_stream() << logging::Logger::prefix(log) << value;
    return logging::Logger::Mode(log, 0, true);
}

template<typename T>
inline logging::Logger::Mode operator<<(logging::Logger::Mode mode, const T &value) {
    if( logging::Logger::available() && (mode.mode_ >= logging::Logger::current_mode()) ) {
        if( (mode.mode_ == logging::Logger::Debug) && (mode.debug_severity_ >= logging::Logger::current_debug_threshold()) ) {
            if( !mode.continuation_ )
                logging::Logger::output_stream() << logging::Logger::prefix(mode.mode_);
            logging::Logger::output_stream() << value;
        } else if( mode.mode_ != logging::Logger::Debug ) {
            if( !mode.continuation_ )
                logging::Logger::output_stream() << logging::Logger::prefix(mode.mode_);
            logging::Logger::output_stream() << value;
        }
    }
    return logging::Logger::Mode(mode.mode_, mode.debug_severity_, true);
}

inline logging::Logger::Mode operator<<(logging::Logger::mode_t log, std::ostream& (*manip)(std::ostream&)) {
    if( logging::Logger::available() && (log >= logging::Logger::current_mode()) )
        manip(logging::Logger::output_stream());
    return logging::Logger::Mode(log, 0, true);
}

inline logging::Logger::Mode operator<<(logging::Logger::Mode mode, std::ostream& (*manip)(std::ostream&)) {
    if( logging::Logger::available() && (mode.mode_ >= logging::Logger::current_mode()) ) {
        if( (mode.mode_ == logging::Logger::Debug) && (mode.debug_severity_ >= logging::Logger::current_debug_threshold()) ) {
            manip(logging::Logger::output_stream());
        } else if( mode.mode_ != logging::Logger::Debug ) {
            manip(logging::Logger::output_stream());
        }
    }
    return logging::Logger::Mode(mode.mode_, mode.debug_severity_, true);
}

#endif

