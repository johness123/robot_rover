~ Guide for my future self

# How to install vcpkg on WSL

## 📦 Vcpkg Setup & Installation

This project uses [vcpkg](https://github.com/microsoft/vcpkg) as the C++ package manager. Follow the steps below to install dependencies.

### Prerequisites

Before you begin, ensure you have the following installed:
*   [Git](https://git-scm.com/)
*   [CMake](https://cmake.org/download/) (version 3.15 or higher)
*   A C++ compiler (Visual Studio 2015+ on Windows, GCC/Clang on Linux/macOS)

### 1. Clone the vcpkg repository

You need to clone the vcpkg repository to your local machine. It is recommended to clone it outside of your project directory so multiple projects can share the same vcpkg instance.

**Windows:**
```bash
git clone https://github.com/microsoft/vcpkg.git 
```
**Linux/macOS:**

```Bash
git clone https://github.com/microsoft/vcpkg.git ~/vcpkg
```

## 2. Run the bootstrap script
Initialize vcpkg by running the bootstrap script. This will compile the vcpkg executable for your system.

**Windows:**

```DOS
cd C:\dev\vcpkg
.\bootstrap-vcpkg.bat
Linux/macOS:
```

```Bash
cd ~/vcpkg
./bootstrap-vcpkg.sh
```
## 3. Setup Environment Variable (Optional but Recommended)
To make it easier to run vcpkg commands from anywhere, add the vcpkg directory to your system's PATH environment variable. Also, set the VCPKG_ROOT environment variable pointing to your vcpkg installation path.

**Windows (PowerShell):**

```PowerShell
[Environment]::SetEnvironmentVariable("VCPKG_ROOT", "C:\dev\vcpkg", "User")
```
**Linux/macOS (add to your ~/.bashrc or ~/.zshrc):**

```Bash
export VCPKG_ROOT=~/vcpkg
export PATH=$VCPKG_ROOT:$PATH
```
## 4. Build the Project with CMake
This project uses vcpkg in Manifest mode. You don't need to manually install packages using vcpkg install. Instead, CMake will read the vcpkg.json file in the project root and automatically download and build the required dependencies during the configuration step.

Pass the vcpkg toolchain file to CMake when configuring the project:

***Using Command Line:***

```Bash
# Create build directory
mkdir build && cd build

# Configure the project with vcpkg toolchain
cmake .. -DCMAKE_TOOLCHAIN_FILE="$VCPKG_ROOT/scripts/buildsystems/vcpkg.cmake"

# Build the project
cmake --build .
Note: If you are using an IDE like Visual Studio, Visual Studio Code (with CMake Tools extension), or CLion, you can specify the CMAKE_TOOLCHAIN_FILE in the IDE's CMake settings/profiles.
```