#include "WindowsConfig.h"
#include "SKSE/SKSE.h"
#include "Plugin.h"
#include "MotionHooks.h"
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/msvc_sink.h>
#include <windows.h>

namespace
{
void InitializeLogging() noexcept
{
	try {
		auto path = SKSE::log::log_directory();
		if (!path) {
			return;
		}
		*path /= "SVT.log";
		std::shared_ptr<spdlog::sinks::sink> sink;
		if (REX::W32::IsDebuggerPresent()) {
			sink = std::make_shared<spdlog::sinks::msvc_sink_mt>();
		} else {
			sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(path->string(), true);
		}
		auto logger = std::make_shared<spdlog::logger>("SVT", std::move(sink));
		logger->set_level(spdlog::level::info);
		logger->flush_on(spdlog::level::err);
		logger->set_pattern("[%H:%M:%S.%e] [%l] [%t] %v");
		spdlog::set_default_logger(std::move(logger));
	} catch (...) {
		// Retain the existing logger; a file/sink failure must not prevent SKSE registration.
		OutputDebugStringA("SVT: Unable to initialize logging. Plugin loading will continue.\n");
	}
}

void OnMessage(SKSE::MessagingInterface::Message* message)
{
	if (message) {
		SVT::MotionHooks::OnGameMessage(message->type);
	}
	if (message && message->type == SKSE::MessagingInterface::kDataLoaded) {
		SVT::InstallMotion();
		try {
			spdlog::default_logger()->flush();
		} catch (...) {
		}
	}
}
} // namespace

bool SVT::Load(const SKSE::LoadInterface* skse)
{
	SKSE::Init(skse);
	InitializeLogging();
	try {
		SKSE::log::info("{} v{}", kPluginName, SVT_VERSION);
		SKSE::log::info("Game version: {}", REL::Module::get().version().string("."));
		spdlog::default_logger()->flush();
	} catch (...) {
	}
	const bool registered = SKSE::GetMessagingInterface()->RegisterListener(OnMessage);
	if (!registered) {
		try {
			SKSE::log::error("Failed to register the SKSE messaging listener.");
		} catch (...) {
			// A logging failure must not turn failed registration into success.
		}
	}
	return registered;
}

SKSEPluginLoad(const SKSE::LoadInterface* skse)
{
	return SVT::Load(skse);
}
