set_project("pinocchio-bridge")
set_version("0.1.0")
set_languages("cxx17")
set_optimize("fastest")
add_rules("plugin.compile_commands.autoupdate", { outputdir = "." })
-- set_plat("macosx")
-- add_cxxflags("-mmacosx-version-min=15.5")
-- add_ldflags("-mmacosx-version-min=15.5")

-- Libs
-- add_requires("eigen", { system = false })
add_requires("pinocchio", {
    configs = { shared = false, urdf = true, python = false },
    system = false
})
add_requires("urdfdom", { configs = { shared = false }, system = false })


-- Main target
target("pinocchio_bridge")
set_kind("static")
    add_packages("pinocchio", "urdfdom")
    set_pcxxheader("src/pcxx.h")
    add_files("src/**.cpp")
    add_includedirs(".", {public = true})
    add_includedirs("lib", {public = true})
    -- Flags
    add_cxxflags("-Wall", "-Wextra", "-Wpedantic")
    -- set_policy("build.merge_archive", true)
    -- set_policy("build.merge_staticlib", true)

    after_link(function(target)
        import("utils.archive.merge_staticlib")
        local static_libs = {}
        table.insert(static_libs, target:targetfile())
        for _, pkg in pairs(target:pkgs()) do
            for _, libfile in ipairs(pkg:libraryfiles() or {}) do
                if libfile:endswith(".a") or libfile:endswith(".lib") then
                    table.insert(static_libs, libfile)
                end
            end
        end
        merge_staticlib(target, target:targetfile(), static_libs)
        print("[fat-static] merged %d archives into %s", #static_libs, target:targetfile())
    end)


    -- after_build(function(target)
    --     local output = target:targetfile()
    --     local pkg = target:pkg("urdfdom")
    --     local static_libs = {}
    --     for _, f in ipairs(pkg:libraryfiles() or {}) do
    --         if f:endswith(".a") then
    --             table.insert(static_libs, f)
    --         end
    --     end
    --     if #static_libs > 0 then
    --         local args = { "-static", "-o", output, output }
    --         table.join2(args, static_libs)
    --         os.execv("libtool", args)
    --     end
    -- end)
