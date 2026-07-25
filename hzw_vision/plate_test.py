import cv2
import numpy as np
import time
from Cameracontrol import Cameracontrol

def test_get_closer_to_car_center_interactive():
    """
    Interactive test for get_closer_to_car_center function with real-time parameter adjustment
    """
    try:
        # Initialize camera control object (using color camera mode)
        camera = Cameracontrol(0)
        print("Camera initialized successfully")
        
        # Color mapping dictionary for display
        color_name_map = {
            1: "Red",
            2: "Green",
            3: "Blue"
        }
        
        # Initial parameters
        duration = 0.1
        still_threshold = 2
        min_area = 11500
        max_area = 24000
        mode = "single"
        
        # Create parameter adjustment window
        cv2.namedWindow("Parameter Control")
        cv2.createTrackbar("Duration x100", "Parameter Control", int(duration * 100), 200, lambda x: None)
        cv2.createTrackbar("Still Threshold", "Parameter Control", still_threshold, 10, lambda x: None)
        cv2.createTrackbar("Min Area x1000", "Parameter Control", int(min_area/1000), 50, lambda x: None)
        cv2.createTrackbar("Max Area x1000", "Parameter Control", int(max_area/1000), 100, lambda x: None)
        
        print("Key controls:")
        print("  m - Toggle mode (single/every)")
        print("  r - Reset parameters to default")
        print("  q - Quit program")
        print("Use trackbars to adjust other parameters")
        
        running = True
        while running:
            # Get latest parameters from trackbars
            duration = cv2.getTrackbarPos("Duration x100", "Parameter Control") / 100.0
            still_threshold = cv2.getTrackbarPos("Still Threshold", "Parameter Control")
            min_area = cv2.getTrackbarPos("Min Area x1000", "Parameter Control") * 1000
            max_area = cv2.getTrackbarPos("Max Area x1000", "Parameter Control") * 1000
            
            # Display current parameters
            param_frame = np.zeros((200, 600, 3), dtype=np.uint8)
            cv2.putText(param_frame, f"Duration: {duration:.2f}s", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(param_frame, f"Still Threshold: {still_threshold}", (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(param_frame, f"Min Area: {min_area}", (20, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(param_frame, f"Max Area: {max_area}", (20, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(param_frame, f"Mode: {mode}", (20, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(param_frame, "Press 'm' to toggle mode, 'r' to reset, 'q' to quit", (20, 180), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            cv2.imshow("Parameter Control", param_frame)
            
            # Call the function with current parameters
            start_time = time.time()
            color_id, coords, found_flag, still_flag = camera.get_closer_to_car_center(
                duration=duration,
                still_threshold=still_threshold,
                MIN_AREA=min_area,
                MAX_AREA=max_area,
                Mode=mode
            )
            process_time = time.time() - start_time
            
            # Display results
            result_frame = np.zeros((200, 600, 3), dtype=np.uint8)
            cv2.putText(result_frame, f"Process time: {process_time:.3f}s", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(result_frame, f"Found: {found_flag}", (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(result_frame, f"Still: {still_flag}", (20, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            if found_flag:
                color_name = color_name_map.get(color_id, "Unknown")
                cv2.putText(result_frame, f"Color: {color_name} (ID: {color_id})", (20, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                cv2.putText(result_frame, f"Coordinates: x={coords[0]:.1f}, y={coords[1]:.1f}", (20, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            else:
                cv2.putText(result_frame, "No blocks detected", (20, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            cv2.imshow("Results", result_frame)
            
            # Handle key presses
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                running = False
            elif key == ord('m'):
                mode = "every" if mode == "single" else "single"
            elif key == ord('r'):
                # Reset to default values
                duration = 0.5
                still_threshold = 2
                min_area = 5000
                max_area = 55000
                mode = "single"
                
                # Update trackbars
                cv2.setTrackbarPos("Duration x100", "Parameter Control", int(duration * 100))
                cv2.setTrackbarPos("Still Threshold", "Parameter Control", still_threshold)
                cv2.setTrackbarPos("Min Area x1000", "Parameter Control", int(min_area/1000))
                cv2.setTrackbarPos("Max Area x1000", "Parameter Control", int(max_area/1000))
        
        # Clean up resources
        camera.close_cam()
        camera.close_windows()
        cv2.destroyAllWindows()
        print("Test completed, resources released")
        
    except Exception as e:
        print(f"Error during test: {e}")
        # Ensure resources are released
        try:
            camera.close_cam()
            camera.close_windows()
            cv2.destroyAllWindows()
        except:
            pass

if __name__ == "__main__":
    test_get_closer_to_car_center_interactive()
