Dependencies:
	1. CMake
	2. Visual Studio 2017
	3. Install OpenCV 4.2.0:
		- Download opencv 4.2.0 from https://github.com/opencv/opencv/releases
		- Run cmake and:
			1. set the source_code to be the path of opencv, let's call it OPENCV_PATH
			2. set build directory to OPENCV_PATH/build
			3. click on configure and then click yes. At the prompt make sure you choose visual studio 17 and x64 platform.
			4. Disable the following:
				a. BUILD_JAVA
				b. BUILD_OPENCV_java_bindings_generator
				c. BUILD_OPENCV_python_bindings_generator
				d. BUILD_opencv_python_tests
			5. Enable the following:
				a. BUILD_opencv_world
			6. click generate
		6. Go to OPENCV_PATH/build and run powershell:
			a. cmake --build . --config Release --target INSTALL
			b. cmake --build . --config Debug --target INSTALL
		7. Set OPEN_CV System environment variable to open cv directory.
		8. Add to System PATH environment variable %OPEN_CV%\build\install\x64\vc15\bin\
		9. All relevant files for includes and linking are located under %OPEN_CV%\build\install
		10. Pay attention that in order the project to work you must restart visual studio after these changes. 
