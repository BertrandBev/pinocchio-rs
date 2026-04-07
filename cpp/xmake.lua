set_project("pinocchio")
set_languages("cxx20")
set_optimize("fastest")
add_rules("plugin.compile_commands.autoupdate", { outputdir = "." })

-- Libs
add_requires("pinocchio", {
    configs = { shared = false, urdf = true, python = false },
    system = false
})
add_requires("urdfdom", { configs = { shared = false }, system = false })


-- Main target
target("pinocchio")
set_kind("static")
    set_symbols("hidden")
    set_strip("all")
    add_packages("pinocchio", "urdfdom")
    set_pcxxheader("src/pcxx.h")
    add_files("src/**.cpp")
    add_includedirs(".", {public = true})
    add_includedirs("lib", {public = true})
    -- Flags
    add_cxxflags("-Wall", "-Wextra", "-Wpedantic")
    add_cxxflags("-mmacosx-version-min=15.5")
    add_ldflags("-mmacosx-version-min=15.5")

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
