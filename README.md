# GROUP-8_LABORATORY-6

Laboratory Assignment 6: Introduction to Real-Time Operating Systems (RTOS) with FreeRTOS

Learning Objectives:
Understand the basic concepts of an RTOS (tasks, scheduling).
Enable and configure FreeRTOS within STM32CubeIDE.
Create and manage multiple FreeRTOS tasks.
Use RTOS time management features (e.g., vTaskDelay).

Prerequisites:

Completion of previous labs (especially Lab 1 and 3).

An STM32 board with sufficient resources (most modern ones supported by CubeIDE work well).

Assignment Description:
Create a new project in STM32CubeIDE for your board, and enable FreeRTOS middleware. Choose a configuration (e.g., default CMSIS v2). Reference the tutorials on DeepBlueEmbedded covering FreeRTOS integration.

Create at least two separate FreeRTOS tasks.

Task 1: Modify your LED blinking code from Lab 1 to run as a FreeRTOS task. Use vTaskDelay() instead of HAL_Delay(). Make it blink at a specific rate (e.g., 500ms).

Task 2: Modify your button reading code (using interrupts) from Lab 1 to run as a FreeRTOS task. When the button interrupt occurs, set a flag or send a simple notification to this task. The task should then print a message like "Button Pressed!" via UART (using Lab 3 techniques) and toggle a different LED (if available). 
Use vTaskDelay() within this task for brief pauses if needed, or simply wait for the notification.

Observe how both tasks appear to run concurrently.


