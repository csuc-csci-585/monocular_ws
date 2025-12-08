# ROS 2 Workspace Package Versions

This document specifies the exact versions of external packages used in this workspace. Use these references when setting up git submodules to ensure reproducible builds.

## External Package Versions

### depth_anything_v2_ros2
- **Repository**: https://github.com/grupo-avispa/depth_anything_v2_ros2.git
- **Branch**: `main`
- **Commit**: `7cfdc8a261590816c148acbe5fa51131fbf19f4e` 
- **Commit Message**: "Update ROS setup and CI action versions"
- **Date**: Current as of November 24, 2025

### turtlebot3
- **Repository**: https://github.com/ROBOTIS-GIT/turtlebot3.git
- **Branch**: `humble`
- **Commit**: `4ab470dafdeb5a589432adadf7bef066fc2c1c20`
- **Commit Message**: "Merge pull request #1127 from ROBOTIS-GIT/main"
- **Date**: Current as of November 24, 2025

### turtlebot3_simulations
- **Repository**: https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
- **Branch**: `humble`
- **Commit**: `a35a56c8b04877dc89772b598084d8ce648a9023`
- **Commit Message**: "Merge pull request #246 from ROBOTIS-GIT/feature-tb3-manipulation-error-fix"
- **Date**: Current as of November 24, 2025

## Git Submodule Setup Commands

When creating your repository, use these commands to add submodules at the exact versions:

```bash
# Add submodules at specific commits
git submodule add https://github.com/grupo-avispa/depth_anything_v2_ros2.git external_packages/depth_anything_v2_ros2
git submodule add https://github.com/ROBOTIS-GIT/turtlebot3.git external_packages/turtlebot3
git submodule add https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git external_packages/turtlebot3_simulations

# Lock to specific commits
cd external_packages/depth_anything_v2_ros2
git checkout 7cfdc8a261590816c148acbe5fa51131fbf19f4e

cd ../turtlebot3
git checkout 4ab470dafdeb5a589432adadf7bef066fc2c1c20

cd ../turtlebot3_simulations
git checkout a35a56c8b04877dc89772b598084d8ce648a9023

cd ../..
git add .gitmodules external_packages/
git commit -m "Add external packages as submodules at specific versions"
```

## Alternative: .gitmodules Configuration

Your `.gitmodules` file should look like this:

```ini
[submodule "external_packages/depth_anything_v2_ros2"]
	path = external_packages/depth_anything_v2_ros2
	url = https://github.com/grupo-avispa/depth_anything_v2_ros2.git
	branch = main

[submodule "external_packages/turtlebot3"]
	path = external_packages/turtlebot3
	url = https://github.com/ROBOTIS-GIT/turtlebot3.git
	branch = humble

[submodule "external_packages/turtlebot3_simulations"]
	path = external_packages/turtlebot3_simulations
	url = https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
	branch = humble
```

## Local Packages

The following packages are developed locally and included directly in the repository:
- `mypackage` - Local development package (not a submodule)

## Usage for New Users

To clone this workspace with exact package versions:

```bash
# Clone with submodules
git clone --recursive <your-repo-url>
cd <your-workspace>

# If submodules weren't cloned recursively
git submodule update --init --recursive

# Open in VS Code Dev Container
code .
# Click "Reopen in Container" when prompted
```

## Updating Package Versions

To update to newer versions of external packages:

1. Navigate to the submodule directory
2. Fetch and checkout the desired version
3. Commit the submodule pointer update in the main repository

```bash
# Example: Update turtlebot3 to latest humble
cd external_packages/turtlebot3
git fetch origin
git checkout origin/humble
cd ../..
git add external_packages/turtlebot3
git commit -m "Update turtlebot3 to latest humble branch"
```

## Verification

After setup, verify all packages are detected:

```bash
# In the dev container
cd /home/ros/monocular_ws
colcon list
```

You should see all packages from the external repositories plus your local packages.

---
*Last updated: November 24, 2025*
*Generated from workspace at commit time*