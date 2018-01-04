# Enhanced version of coupled FEM and constrained rigid body simulation

## Build/run
The sample uses Vulkan graphics API, and one needs to perform following steps in order to build and run the sample:
1. Download and install [LunarG Vulkan SDK](https://vulkan.lunarg.com/sdk/home) (for the reference, the framework uses 1.0.61.1, but any recent should do);
2. Update `vkEngine\PropertySheet.props` to contain correct SDK installation path, like this:
    ```
    <VulkanSDKRoot>d:\Program Files\VulkanSDK\1.0.61.1</VulkanSDKRoot>
    ```
3. Run `vkEngine.sln` and build as usual.

## Controls:
* `WASD`+`PgDn`/`PgUp`+mouse - control camera, use [shift] to move faster and [ctrl] to move slower
* `q` to pause/unpause
* `p` to perform a single step
* `e` to throw a box
* right mouse button to switch mouse modes (default is camera control mode, alternative is picking mode - use LMB to pick bodies when in alternative mouse mode)

## Description

## License
[Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International Public License](https://creativecommons.org/licenses/by-nc-sa/4.0/legalcode)
