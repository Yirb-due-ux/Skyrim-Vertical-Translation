set_config("skse_xbyak", true)
set_policy("package.install_locally", true)
add_requireconfs("xbyak", { version = "v7.06", override = true })
includes("external/CommonLibSSE")

set_project("SVT")
local pluginVersion = "0.1.0"
set_version(pluginVersion)
set_languages("c++23")
set_warnings("allextra")

add_rules("mode.debug", "mode.releasedbg")

target("svt_zycore")
    set_kind("static")
    set_default(false)
    set_languages("c11")
    add_files("external/Zydis/dependencies/zycore/src/**.c")
    add_includedirs("external/Zydis/dependencies/zycore/include", { public = true })
    add_defines("ZYCORE_STATIC_BUILD", { public = true })

target("svt_zydis")
    set_kind("static")
    set_default(false)
    set_languages("c11")
    add_deps("svt_zycore")
    add_files("external/Zydis/src/*.c")
    add_includedirs("external/Zydis/include", { public = true })
    add_includedirs("external/Zydis/src")
    add_defines("ZYDIS_STATIC_BUILD", { public = true })

target("svt_safetyhook")
    set_kind("static")
    set_default(false)
    add_deps("svt_zydis")
    add_files("external/SafetyHook/src/allocator.cpp", "external/SafetyHook/src/inline_hook.cpp",
              "external/SafetyHook/src/os.windows.cpp", "external/SafetyHook/src/utility.cpp")
    add_includedirs("external/SafetyHook/include", { public = true })

target("SVT")
    set_kind("shared")
    add_defines('SVT_VERSION="' .. pluginVersion .. '"')
    add_deps("commonlibsse", "svt_safetyhook")
    add_rules("commonlibsse.plugin", {
        name = "Skyrim Vertical Translation",
        author = "Yirb-due-ux",
        description = "Vertical root motion with native collision for Skyrim SE 1.5.97",
        install = false,
        package = false,
        autoinstall = false
    })
    add_files(
        "src/main.cpp",
        "src/MotionHooks.cpp",
        "src/Installation.cpp",
        "src/PhysicsHooks.cpp",
        "src/RigidBodyMotion.cpp",
        "src/Hooks.cpp"
    )
    set_pcxxheader("src/PCH.h")
