--
-- premake4 file to build TerrainRL
-- Copyright (c) 2009-2015 Glen Berseth
-- See license.txt for complete license.
--

local action = _ACTION or ""
local todir = "./" .. action

function file_exists(name)
   local f=io.open(name,"r")
   if f~=nil then io.close(f) return true else return false end
end

local glfwLocation = "include/glfw/"

solution "CPSC426"
	configurations { 
		"Debug",
		"Release"
	}
	
	platforms {
		"x32", 
		"x64"
	}
	-- location (todir)

	-- extra warnings, no exceptions or rtti
	flags { 
		-- "ExtraWarnings",
--		"FloatFast",
--		"NoExceptions",
--		"NoRTTI",
		"Symbols"
	}
	libdirs { 
		"lib",
	}
	-- defines { "ENABLE_GUI", "ENABLE_GLFW" }

	-- debug configs
	configuration { "Debug*"}
		defines { "DEBUG" }
		flags {
			"Symbols",
			Optimize = Off
		}
		targetdir ( "./x64/Debug" )
 
 	-- release configs
	configuration {"Release*"}
		defines { "NDEBUG" }
		flags { "Optimize" }
		targetdir ( "./x64/Release" )

	-- windows specific
	configuration {"windows"}
		defines { "WIN32", "_WINDOWS" }
		libdirs { "lib" }
		-- flags {"StaticRuntime"}

if os.get() == "windows" then
		debugdir "./"
end
	configuration {"windows", "Debug*"}
		targetdir ( "./x64/Debug" )
	configuration {"windows", "Release*"}
		targetdir ( "./x64/Release" )

	configuration { "linux", "Debug*", "gmake"}
        buildoptions { "-ggdb -fPIC" }
		linkoptions { 
			-- "-stdlib=libc++" ,
			"-Wl,-rpath," .. path.getabsolute("lib")
		}
		targetdir ( "./x64/Debug" )

	configuration { "linux", "Release*", "gmake"}
        buildoptions { "-ggdb -fPIC" }
		linkoptions { 
			-- "-stdlib=libc++" ,
			"-Wl,-rpath," .. path.getabsolute("lib")
		}
		targetdir ( "./x64/Release" )

	configuration { "macosx" }
        	buildoptions {  "-ggdb -fPIC" }
		linkoptions { 
			"-stdlib=libc++" ,
			"-Wl,-rpath," .. path.getabsolute("lib")
		}
		links {
	        "OpenGL.framework",
        }
		targetdir ( "./x64/Debug" )
      
	if os.get() == "macosx" then
		premake.gcc.cc = "clang"
		premake.gcc.cxx = "clang++"
		-- buildoptions("-std=c++0x -ggdb -stdlib=libc++" )
	end


project "CPSC426"
	language "C++"
	kind "ConsoleApp"

	files { 
		-- Source files for this project
		"*.cpp",
		"*.h"
	}
	excludes 
	{ -- Files to exclude from this project but match the above regular expression(s)
	}	
	includedirs { 
		"./",
		"include/eigen",
		"include/glfw/include",
		"include/nanovg/src",
		"jsoncpp/include",
		"scenarios",
		"util"
	}
	links {
		"glfw",
		"cpsc426Util",
		"nanoGUI",
		"jsoncpp",
		"cpsc426Render",
		"nanovg",
		"cpsc426Scenario",
	}

	defines { -- preprocessor defines for this project
		"_CRT_SECURE_NO_WARNINGS",
		"_SCL_SECURE_NO_WARNINGS",
	}

	buildoptions("-std=c++0x -ggdb" ) -- allows for debuging	

	-- linux library cflags and libs
	configuration { "linux", "gmake" }
		buildoptions { 
			"`pkg-config --cflags gl`",
			"`pkg-config --cflags glu`" 
		}
		linkoptions { 
			"-Wl,-rpath," .. path.getabsolute("lib") ,
			"`pkg-config --libs gl`",
			"`pkg-config --libs glu`" 
		}
		libdirs { 
			-- "lib",
		}
		links {
			"X11",
			"Xrandr",
			"Xi",
			"Xxf86vm",
			"Xinerama",
			"Xcursor",
		}
		
		includedirs { 

		}
		defines {
			"_LINUX_",
		}
			-- debug configs
		configuration { "linux", "Debug*", "gmake"}
			links {
				"dl",
				"pthread",
			}
	 
	 	-- release configs
		configuration { "linux", "Release*", "gmake"}
			defines { "NDEBUG" }
			links {
				"dl",
				"pthread",
			}

	-- windows library cflags and libs
	configuration { "windows" }
		-- libdirs { "lib" }
		linkoptions  { 
		}
		files {
			-- "include/glad/src/*.c",	
		}
		defines {
			"_USE_MATH_DEFINES",
			"NANOGUI_GLAD",
			"NANOGUI_EIGEN_DONT_ALIGN",
			"GLAD_GLAPI_EXPORT" -- Because each library must have the same calling convention
		}
		includedirs { 
			"include/glad/include",
		}	
		
		libdirs { 
			"lib",
		}
		
		-- release configs
		configuration { "windows", "Debug*"}
			defines { "DEBUG" }
			links { 
				"opengl32",
				"glu32",
				"glew32",
			}

		-- release configs
		configuration { "windows", "Release*"}
			defines { "NDEBUG" }
			links { 
				"opengl32",
				"glu32",
				"glew32",
			}

	-- mac includes and libs
	configuration { "macosx" }
		kind "ConsoleApp" -- xcode4 failes to run the project if using WindowedApp
		-- includedirs { "/Library/Frameworks/SDL.framework/Headers" }
		buildoptions { "-Wunused-value -Wshadow -Wreorder -Wsign-compare -Wall" }
		linkoptions { 
			"-Wl,-rpath," .. path.getabsolute("lib") ,
		}
		links { 
			"OpenGL.framework", 
			"Cocoa.framework",
			"dl",
			"pthread"
		}

project "cpsc426Util"
	language "C++"
	kind "SharedLib"

	files { 
		-- Source files for this project
		"util/*.h",
		"util/*.cpp"
	}
	excludes 
	{
	}	
	includedirs { 
		"./",
		"util",
		"include/eigen",
	}

	

	defines {
		"_CRT_SECURE_NO_WARNINGS",
		"_SCL_SECURE_NO_WARNINGS",
	}

	buildoptions("-std=c++0x -ggdb" )	

	-- linux library cflags and libs
	configuration { "linux", "gmake" }
		targetdir ( "./lib" )
		buildoptions { 
			"`pkg-config --cflags gl`",
			"`pkg-config --cflags glu`" 
		}
		linkoptions { 
			"-Wl,-rpath," .. path.getabsolute("lib") ,
			"`pkg-config --libs gl`",
			"`pkg-config --libs glu`" 
		}
		libdirs { 
			-- "lib",
		}
		
		includedirs { 

		}
		defines {
			"_LINUX_",
		}
			-- debug configs
		configuration { "linux", "Debug*", "gmake"}
			links {
				"dl",
				"pthread",
			}
	 
	 	-- release configs
		configuration { "linux", "Release*", "gmake"}
			defines { "NDEBUG" }
			links {
				"dl",
				"pthread",
			}

	-- windows library cflags and libs
	configuration { "windows" }
		-- libdirs { "lib" }
		linkoptions  { 
			-- "`pkg-config --cflags glu`" 
		}
		defines {
			"_USE_MATH_DEFINES",
			"_WIN32",
			"PLUGIN_API",
		}
		includedirs { 
		}	
		
		libdirs { 
		}
		
		-- release configs
		configuration { "windows", "Debug*"}
			defines { "DEBUG" }
			links { 
			}

		-- release configs
		configuration { "windows", "Release*"}
			defines { "NDEBUG" }
			links { 
			}

	-- mac includes and libs
	configuration { "macosx" }
		-- includedirs { "/Library/Frameworks/SDL.framework/Headers" }
        targetdir ( "./lib" )
		buildoptions { "-Wunused-value -Wshadow -Wreorder -Wsign-compare -Wall" }
		linkoptions { 
			"-Wl,-rpath," .. path.getabsolute("lib") ,
		}
		links {
			"dl",
			"pthread"
		}
        -- removebuildoptions ("-std=c++0x")
        linkoptions { 
		"-framework OpenGL", 
		"-framework Cocoa", 
		"-framework IOKit", 
        "-framework CoreVideo",
                "-install_name @rpath/libcpsc426Util.dylib"
        }
        buildoptions {
                        "-fPIC",
                }


project "cpsc426Scenario"
	language "C++"
	kind "SharedLib"

	files { 
		-- Source files for this project
		--"util/*.h",
		"scenarios/*.cpp",
		"scenarios/*.h",
		"Cloth.cpp",
		"Cloth.h",
	}
	excludes 
	{
	}	
	includedirs { 
		"./",
		"scenario",
		"include/eigen",
		"include/glfw/include",
		"include/nanovg/src",
		"jsoncpp/include",
	}
	links {
		"glfw",
		"jsoncpp",
		"nanoGUI",
		"cpsc426Util",
		"cpsc426Render",
	}

	

	defines {
		"_CRT_SECURE_NO_WARNINGS",
		"_SCL_SECURE_NO_WARNINGS",
	}

	buildoptions("-std=c++0x -ggdb" )	

	-- linux library cflags and libs
	configuration { "linux", "gmake" }
		targetdir ( "./lib" )
		buildoptions { 
			"`pkg-config --cflags gl`",
			"`pkg-config --cflags glu`" 
		}
		linkoptions { 
			"-Wl,-rpath," .. path.getabsolute("lib") ,
			"`pkg-config --libs gl`",
			"`pkg-config --libs glu`" 
		}
		libdirs { 
			-- "lib",
		}
		
		includedirs { 

		}
		defines {
			"_LINUX_",
		}
			-- debug configs
		configuration { "linux", "Debug*", "gmake"}
			links {
				"dl",
				"pthread",
			}
	 
	 	-- release configs
		configuration { "linux", "Release*", "gmake"}
			defines { "NDEBUG" }
			links {
				"dl",
				"pthread",
			}

	-- windows library cflags and libs
	configuration { "windows" }
		-- libdirs { "lib" }
		linkoptions  { 
			-- "`pkg-config --cflags glu`" 
		}
		files {
			-- "include/glad/src/*.c",	
		}
		defines {
			"_USE_MATH_DEFINES",
			"NANOGUI_GLAD",
			"_WIN32",
			"PLUGIN_API",
			"GLAD_GLAPI_EXPORT" -- Because each library must have the same calling convention
		}
		includedirs { 
			"include/glad/include",
		}	
		
		libdirs { 
		}
		
		-- release configs
		configuration { "windows", "Debug*"}
			defines { "DEBUG" }
			links { 
				"opengl32",
				"glu32",
				"glew32",
			}

		-- release configs
		configuration { "windows", "Release*"}
			defines { "NDEBUG" }
			links { 
				"opengl32",
				"glu32",
				"glew32",
			}

	-- mac includes and libs
	configuration { "macosx" }
        targetdir ( "./lib" )
		-- includedirs { "/Library/Frameworks/SDL.framework/Headers" }
		buildoptions { "-Wunused-value -Wshadow -Wreorder -Wsign-compare -Wall" }
		linkoptions { 
			"-Wl,-rpath," .. path.getabsolute("lib") ,
		}
		links { 
			"OpenGL.framework", 
			"Cocoa.framework",
			"dl",
			"pthread"
		}
                linkoptions {
                "-install_name @rpath/libcpsc426Scenario.dylib"
        }
        buildoptions {
            "-fPIC"
            }




project "cpsc426Render"
	language "C++"
	kind "SharedLib"

	files { 
		-- Source files for this project
		--"util/*.h",
		"render/*.cpp",
		"render/*.h"
	}
	excludes 
	{
	}	
	includedirs { 
		"./",
		"./render",
		"include/eigen",
		"include/glfw/include",
		"include/nanovg/src",
		"jsoncpp/include",
		"scenarios",
		"util",
	}
	links {
		"glfw",
		"cpsc426Util",
		"nanoGUI",
	}


	

	defines {
		"_CRT_SECURE_NO_WARNINGS",
		"_SCL_SECURE_NO_WARNINGS",
	}

	buildoptions("-std=c++0x -ggdb" )	

	-- linux library cflags and libs
	configuration { "linux", "gmake" }
		targetdir ( "./lib" )
		buildoptions { 
			"`pkg-config --cflags gl`",
			"`pkg-config --cflags glu`" 
		}
		linkoptions { 
			"-Wl,-rpath," .. path.getabsolute("lib") ,
			"`pkg-config --libs gl`",
			"`pkg-config --libs glu`" 
		}
		libdirs { 
			-- "lib",
		}
		
		includedirs { 

		}
		defines {
			"_LINUX_",
		}
			-- debug configs
		configuration { "linux", "Debug*", "gmake"}
			links {
				"dl",
				"pthread",
			}
	 
	 	-- release configs
		configuration { "linux", "Release*", "gmake"}
			defines { "NDEBUG" }
			links {
				"dl",
				"pthread",
			}

	-- windows library cflags and libs
	configuration { "windows" }
		-- libdirs { "lib" }
		linkoptions  { 
			-- "`pkg-config --cflags glu`" 
		}
		files {
			-- "include/glad/src/*.c",	
		}
		defines {
			"_USE_MATH_DEFINES",
			"NANOGUI_GLAD",
			"_WIN32",
			"PLUGIN_API",
			"GLAD_GLAPI_EXPORT",
		}
		includedirs { 
			"include/glad/include",
		}	
		
		libdirs { 
		}
		
		-- release configs
		configuration { "windows", "Debug*"}
			defines { "DEBUG" }
			links { 
			}

		-- release configs
		configuration { "windows", "Release*"}
			defines { "NDEBUG" }
			links { 
			}

	-- mac includes and libs
	configuration { "macosx" }
		targetdir ( "./lib" )
        -- includedirs { "/Library/Frameworks/SDL.framework/Headers" }
		buildoptions { "-Wunused-value -Wshadow -Wreorder -Wsign-compare -Wno-c++11-narrowing -Wall" }
		linkoptions { 
			"-Wl,-rpath," .. path.getabsolute("lib") ,
		}
		links { 
			"OpenGL.framework", 
			"Cocoa.framework",
			"dl",
			"pthread"
		}
        linkoptions {
                "-install_name @rpath/libcpsc426Render.dylib"
        }
        buildoptions {
            "-fPIC"
            }


project "jsoncpp"
	language "C++"
	kind "SharedLib"

	files { 
		-- Source files for this project
		--"util/*.h",
		"jsoncpp/src/lib_json/*.cpp",
		"jsoncpp/src/lib_json/*.h",
	}
	excludes 
	{
	}	
	includedirs { 
		"./jsoncpp/include",
	}
	links {
	}

	

	defines {
		"_CRT_SECURE_NO_WARNINGS",
		"_SCL_SECURE_NO_WARNINGS",
	}

	buildoptions("-std=c++0x -ggdb" )	

	-- linux library cflags and libs
	configuration { "linux", "gmake" }

		targetdir ( "./lib" )

		buildoptions { 
		}
		linkoptions { 
			"-Wl,-rpath," .. path.getabsolute("lib") ,
		}
		libdirs { 
			-- "lib",
		}
		
		includedirs { 
		}
		defines {
			"_LINUX_",
		}
			-- debug configs
		configuration { "linux", "Debug*", "gmake"}
			links {
			}
	 
	 	-- release configs
		configuration { "linux", "Release*", "gmake"}
			defines { "NDEBUG" }
			links {
			}

	-- windows library cflags and libs
	configuration { "windows" }
		-- libdirs { "lib" }
		linkoptions  { 
		}
		defines {
			"_USE_MATH_DEFINES",
			"_MSC_VER",
			"JSON_DLL_BUILD",
		}
		includedirs { 
		}	
		
		libdirs { 
		}
		
		-- release configs
		configuration { "windows", "Debug*"}
			defines { "DEBUG" }
			links { 
			}

		-- release configs
		configuration { "windows", "Release*"}
			defines { "NDEBUG" }
			links { 
			}

	-- mac includes and libs
	configuration { "macosx" }
        targetdir ( "./lib" )
		-- includedirs { "/Library/Frameworks/SDL.framework/Headers" }
		buildoptions { "-Wunused-value -Wshadow -Wreorder -Wsign-compare -Wall" }
		linkoptions { 
			"-Wl,-rpath," .. path.getabsolute("lib") ,
		}
		links { 
			"OpenGL.framework", 
			"Cocoa.framework",
			"dl",
			"pthread"
		}
        linkoptions {
                "-install_name @rpath/libjsoncpp.dylib"
        }
        buildoptions {
            "-fPIC"
            }


project "nanoGUI"
	language "C++"
	kind "SharedLib"

	files { 
		-- Source files for this project
		"nanogui/src/*.cpp",
		"nanogui/*.h",
	}
	excludes 
	{
		"nanogui/src/example2.cpp",
		"nanogui/src/example3.cpp",
		"nanogui/src/example4.cpp",
	}	
	includedirs {
		"./",
		"nanogui", 
		"include/eigen",
		"include/glfw/include",
		"include/nanovg/src",
	}
	links {
		-- "terrainrlSim",
		"glfw",
		"nanovg",
	}

	defines {
		"_CRT_SECURE_NO_WARNINGS",
		"_SCL_SECURE_NO_WARNINGS",
	}

	buildoptions("-std=c++0x -ggdb" )	

	-- linux library cflags and libs
	configuration { "linux", "gmake" }

		targetdir ( "./lib" )
		buildoptions { 
			"`pkg-config --cflags gl`",
			"`pkg-config --cflags glu`" 
		}
		linkoptions { 
			"-Wl,-rpath," .. path.getabsolute("lib") ,
			"`pkg-config --libs gl`",
			"`pkg-config --libs glu`" 
		}
		libdirs { 
			-- "lib",
		}
		
		includedirs { 
		}
		defines {
			"_LINUX_",
		}
			-- debug configs
		configuration { "linux", "Debug*", "gmake"}
			links {
				"X11",
				"dl",
				"pthread",
			}
	 
	 	-- release configs
		configuration { "linux", "Release*", "gmake"}
			defines { "NDEBUG" }
			links {
				"X11",
				"dl",
				"pthread",
			}

	-- windows library cflags and libs
	configuration { "windows" }
		-- libdirs { "lib" }
		linkoptions  { 
		}
		defines {
			"_USE_MATH_DEFINES",
			"NANOGUI_EIGEN_DONT_ALIGN",
			"NANOGUI_GLAD",
			"NOMINMAX",
			"_WIN32",
			"NANOGUI_BUILD",
			"NANOGUI_SHARED",
		}
		includedirs { 
			"include/glad/include",
		}	
		files {
			"include/glad/include/glad/*.h",
		}
		
		libdirs { 
		}
		
		-- release configs
		configuration { "windows", "Debug*"}
			defines { "DEBUG" }
			links { 
				"opengl32",
				"glu32",
				"glew32",
			}

		-- release configs
		configuration { "windows", "Release*"}
			defines { "NDEBUG" }
			links { 
				"opengl32",
				"glu32",
				"glew32",
			}

	-- mac includes and libs
	configuration { "macosx" }
        targetdir ( "./lib" )
		-- includedirs { "/Library/Frameworks/SDL.framework/Headers" }
		buildoptions { "-Wunused-value -Wshadow -Wreorder -Wsign-compare -Wall" }
		linkoptions { 
			"-Wl,-rpath," .. path.getabsolute("lib") ,
		}
        files {
            "nanogui/src/*.mm",
        }
		links { 
			"OpenGL.framework", 
			"Cocoa.framework",
			"dl",
			"pthread"
		}
        linkoptions {
                "-install_name @rpath/libnanoGUI.dylib"
        }
        buildoptions {
            "-fPIC"
            }

project "nanovg"
	language "C++"
	kind "SharedLib"

	files { 
		-- Source files for this project
		"include/nanovg/src/*.cpp",
		"include/nanovg/src/*.c",
	}
	excludes 
	{
	}	
	includedirs {
		"./",
		"inlucde/nanovg", 
		"include/eigen",
		"include/glfw/include",
		"include/nanovg/src",
	}
	links {
		-- "terrainrlSim",
		"glfw",
	}

	defines {
		"_CRT_SECURE_NO_WARNINGS",
		"_SCL_SECURE_NO_WARNINGS",
	}

	buildoptions("-ggdb" )

	-- linux library cflags and libs
	configuration { "linux", "gmake" }

		targetdir ( "./lib" )
		buildoptions { 
			"`pkg-config --cflags gl`",
			"`pkg-config --cflags glu`" 
		}
		linkoptions { 
			"-Wl,-rpath," .. path.getabsolute("lib") ,
			"`pkg-config --libs gl`",
			"`pkg-config --libs glu`" 
		}
		libdirs { 
			-- "lib",
		}
		
		includedirs { 
		}
		defines {
			"_LINUX_",
		}
			-- debug configs
		configuration { "linux", "Debug*", "gmake"}
			links {
				"X11",
				"dl",
				"pthread",
			}
	 
	 	-- release configs
		configuration { "linux", "Release*", "gmake"}
			defines { "NDEBUG" }
			links {
				"X11",
				"dl",
				"pthread",
			}

	-- windows library cflags and libs
	configuration { "windows" }
		-- libdirs { "lib" }
		linkoptions  { 
		}
		defines {
			"_USE_MATH_DEFINES",
			"NVG_SHARED",
			"_WIN32",
			"NVG_BUILD",
		}
		includedirs { 
			-- "include/glad/include",
		}	
		
		libdirs { 
		}
		
		-- release configs
		configuration { "windows", "Debug*"}
			defines { "DEBUG" }
			links { 
				"opengl32",
				"glu32",
				"glew32",
			}

		-- release configs
		configuration { "windows", "Release*"}
			defines { "NDEBUG" }
			links { 
				"opengl32",
				"glu32",
				"glew32",
			}

	-- mac includes and libs
	configuration { "macosx" }
        targetdir ( "./lib" )
		-- includedirs { "/Library/Frameworks/SDL.framework/Headers" }
		buildoptions { "-Wunused-value -Wshadow -Wreorder -Wsign-compare -Wall" }
		linkoptions { 
			"-Wl,-rpath," .. path.getabsolute("lib") ,
		}
		links { 
			"OpenGL.framework", 
			"Cocoa.framework",
			"dl",
			"pthread"
		}
        linkoptions {
                "-install_name @rpath/libnanovg.dylib"
        }
        buildoptions {
            "-fPIC"
            }

project "glfw"
	kind "SharedLib"
    	language "C++"
   	includedirs { 
		glfwLocation .. "include",
		glfwLocation .. "src",
		-- glfwLocation .. "lib/x11"
	}
	files { 
	--	glfwLocation .. "lib/x11/*.h",
	--	glfwLocation .. "lib/x11/*.c",
		glfwLocation .. "src/init.c",
		glfwLocation .. "src/input.c",
		glfwLocation .. "src/monitor.c",
		glfwLocation .. "src/context.c",
		glfwLocation .. "src/window.c",
		-- glfwLocation .. "src/glx_context.c",
		glfwLocation .. "src/xkb_unicode.c",
		glfwLocation .. "src/vulkan*.c",
	}
	defines { "GLFW_BUILD_DLL" }
	

    configuration {"linux"}
		targetdir ( "./lib" )
        files { 
			glfwLocation .. "src/x11*.c",
			glfwLocation .. "src/linux*.c",
			glfwLocation .. "src/posix*.c",
			glfwLocation .. "src/glx_context.c",
            glfwLocation .. "src/egl_context.c",
			-- glfwLocation .. "x11/*.h" 
		}
        defines { 
			"_GLFW_USE_LINUX_JOYSTICKS", 
			"_GLFW_HAS_XRANDR", 
			"_GLFW_HAS_PTHREAD", 
			"_GLFW_HAS_SCHED_YIELD", 
			"_GLFW_HAS_GLXGETPROCADDRESS", 
			"_GLFW_X11",
		 }
        links { 
			"pthread",
			"Xrandr",
			"X11",
			"Xi",
			-- "gl",
			"Xcursor",
		}

        
        buildoptions { 
			"-pthread",
			"`pkg-config --cflags gl`",
			"`pkg-config --cflags glu`",  
			"-fPIC",
		}
       
    configuration {"windows"}
        files { 
			glfwLocation .. "src/win32*.c",
			"include/glad/src/*.c",
			"include/glad/include/glad/*.h",
			glfwLocation .. "src/wgl_context.c",
			glfwLocation .. "src/*.h",
			glfwLocation .. "include/GLFW/*.h",
			glfwLocation .. "src/egl_context.c",
			-- glfwLocation .. "x11/*.h" 
		}
		includedirs { 
			"include/glad/include",
			-- glfwLocation .. "lib/x11"
		}
		links {
			"opengl32"
		}
        defines { 
        	"_GLFW_USE_LINUX_JOYSTICKS", 
        	"_GLFW_HAS_XRANDR", 
        	"_GLFW_HAS_PTHREAD",
        	"_GLFW_HAS_SCHED_YIELD", 
        	"_GLFW_HAS_GLXGETPROCADDRESS",
        	"_GLFW_WIN32",
        	"_WIN32",
        	"_GLFW_BUILD_DLL",
        	-- GLAD STUFF
        	"GLAD_GLAPI_EXPORT",
        	"WIN32",
        	"GLAD_GLAPI_EXPORT_BUILD",
        	
        }
       
    configuration {"macosx"}
		targetdir ( "./lib" )
        files { 
        	glfwLocation .. "src/cocoa*.c",
            glfwLocation .. "src/cocoa*.m",
			-- glfwLocation .. "src/linux*.c",
			-- glfwLocation .. "src/posix*.c",
			glfwLocation .. "src/nsgl*.m",
            glfwLocation .. "src/posix_tls.c",

		}
        includedirs { 
        	glfwLocation .. "lib/cocoa" 
        }
        defines {
        	"_GLFW_COCOA",
        }
        links { 
			"pthread",
		}

        
--	removebuildoptions "-std=c++0x"
        linkoptions { 
		"-framework OpenGL", 
		"-framework Cocoa", 
		"-framework IOKit", 
        "-framework CoreVideo",
                "-install_name @rpath/libglfw.dylib"
	}
	buildoptions {
                        "-fPIC",
                }

