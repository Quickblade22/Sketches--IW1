// (c) 2017 Blai Bonet

#include "logger.h"

std::ostream* logging::Logger::output_stream_ = nullptr;
logging::Logger::mode_t logging::Logger::current_mode_ = logging::Logger::Silent;
int logging::Logger::current_debug_threshold_ = 0;
bool logging::Logger::use_color_ = false;

