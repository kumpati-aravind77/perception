'''
    This File is to Run the Scenarios simulated through Carla. 

    Author:
        Sumukha B G
        AI23MTECH14006@iith.ac.in
'''

#Packages for Carla
import carla
from carla import ColorConverter as cc
from agents.navigation.basic_agent import BasicAgent

#Packages for YOLO
import cv2
import numpy as np
import time
from ultralytics import YOLO

#Packages for GUI
import tkinter as tk
import tkinter.font as tkFont
from tkinter import messagebox

# Set the port for the Traffic Manager
import os 
os.environ['TM_PORT'] = '8001'

class ZEDCamera:
    def __init__(self, world, blueprint_library, vehicle):
        """
        Initializes the ZEDCamera class.

        Args:
            world (carla.World): The Carla world object.
            blueprint_library (carla.BlueprintLibrary): The Carla blueprint library object.
            vehicle (carla.Vehicle): The Carla vehicle object.

        Attributes:
            world (carla.World): The Carla world object.
            blueprint_library (carla.BlueprintLibrary): The Carla blueprint library object.
            vehicle (carla.Vehicle): The Carla vehicle object.
            rgb_camera (carla.Actor): The RGB camera actor.
            depth_camera (carla.Actor): The depth camera actor.
            latest_rgb_image (numpy.ndarray): The latest RGB image captured by the camera.
            latest_depth_image (numpy.ndarray): The latest depth image captured by the camera.
        """
        # Store the world, blueprint library, and vehicle
        self.world = world
        self.blueprint_library = blueprint_library
        self.vehicle = vehicle
        
        # Create RGB camera
        rgb_bp = self.blueprint_library.find('sensor.camera.rgb')
        rgb_bp.set_attribute('image_size_x', '800')  # Set image width
        rgb_bp.set_attribute('image_size_y', '600')  # Set image height
        rgb_bp.set_attribute('fov', '90')  # Set field of view
        camera_transform = carla.Transform(carla.Location(x=0, y=0, z=3.0)) #Location of where to add the camera in Vehicle

        # Spawn the camera and attach it to the vehicle
        self.rgb_camera = self.world.spawn_actor(rgb_bp, camera_transform, attach_to=self.vehicle)
        
        # Create depth camera
        depth_bp = self.blueprint_library.find('sensor.camera.depth')
        depth_bp.set_attribute('image_size_x', '800')  # Set image width
        depth_bp.set_attribute('image_size_y', '600')  # Set image height
        self.depth_camera = self.world.spawn_actor(depth_bp, camera_transform, attach_to=self.vehicle)
        
        # Connect callbacks for the sensor data
        self.rgb_camera.listen(self.process_rgb_image)
        self.depth_camera.listen(self.process_depth_image)
        
        # Variables to store the latest RGB and depth images
        self.latest_rgb_image = None
        self.latest_depth_image = None

    def process_rgb_image(self, image):
        """
        Process the RGB image.

        Args:
            image: The RGB image to be processed.

        Returns:
            None
        """
        # Convert RGB image to numpy array
        self.latest_rgb_image = np.frombuffer(image.raw_data, dtype=np.uint8)
        self.latest_rgb_image = self.latest_rgb_image.reshape((image.height, image.width, 4))[:, :, :3]  # Convert to RGB only

    def process_depth_image(self, image):
        """
        Process the depth image and calculate the depth values.

        Args:
            image: The depth image to be processed.

        Returns:
            None
        """
        # Using the ColorConverter of Carla to convert into Depth Image Array
        image.convert(cc.Depth)
        depth_rgb = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        depth_rgb = np.reshape(depth_rgb, (image.height, image.width, 4))

        # Calculate depth from the encoded RGB values
        R = depth_rgb[:, :, 0].astype(np.float32)
        G = depth_rgb[:, :, 1].astype(np.float32)
        B = depth_rgb[:, :, 2].astype(np.float32)
        depth = (R + G * 256 + B * 256 * 256) / (256 * 256 * 256 - 1)
        depth = 1000 * depth # Conversion to Meters

        # Added to avoid Division by Zero Error
        # depth = np.clip(depth, 0.01, np.max(depth))

        self.latest_depth_image = depth

    def get_zed_camera_image(self):
        """
        Get the RGB and depth images from the ZED camera.

        Returns:
            tuple: A tuple containing the latest RGB image and the latest depth image.
        """
        return self.latest_rgb_image, self.latest_depth_image

    def destroy(self):
        """
        Destroys the object and cleans up associated resources.

        This method should be called when you are done using the object to ensure proper cleanup of sensors.

        Returns:
            None
        """
        # Clean up sensors when done
        self.rgb_camera.destroy()
        self.depth_camera.destroy()

def load_yolo_model(pretrained=True, path="Simulation(CARLA)/models_checkpoint/collision_warning_yolov8.pt"):
    """
    Load the YOLO model.

    Args:
        pretrained (bool): Whether to load the pretrained model or not. Default is True.
        path (str): The path to the YOLO model checkpoint file. Default is "models_checkpoint/collision_warning_yolov8.pt".

    Returns:
        model: The loaded YOLO model.

    """
    # Ensure YOLO model uses CUDA
    if pretrained:
        model = YOLO("yolov8n.pt", verbose=False)
    else:
        model = YOLO(path, verbose=False)
    return model

def process_and_display_images(zed_camera, yolo_model, title):
    """
    Process and display images from the ZED camera using the YOLO model.

    Args:
        zed_camera: The ZED camera object.
        yolo_model: The YOLO model for object detection.
        title: The title of the window to display the processed images.

    Returns:
        A tuple containing two values:
        - A boolean indicating whether the function should continue processing images.
        - An integer indicating the safety level (0: safe, 1: caution, 2: danger).
    """
    safe_to_proceed = 0  # Default to safe
    collision_warning_classes = ['person', 'bicycle', 'car', 'motorcycle', 'bus', 'commercial vehicle', 'truck', 'animal', 'pothole', 'speed bump']
    classes_to_be_detected = ['car','person', 'motorcycle', 'bicycle']
    rgb_image, depth_image = zed_camera.get_zed_camera_image()
    if rgb_image is not None and depth_image is not None:
        results = yolo_model(rgb_image, verbose=False)
        frame = rgb_image.copy()
        for box in results[0].boxes:
            cls = results[0].names[box.cls[0].item()]
            if cls in collision_warning_classes:
                cords = box.xyxy[0].tolist()
                x1, y1, x2, y2 = [round(x) for x in cords]

                # Calculate the minimum depth within the bounding box
                box_depth = np.min(depth_image[y1:y2, x1:x2])

                #Error Calculation (Use only when an error is Observed)
                # if box_depth < threshold :
                #     box_depth += error

                confidence = round(box.conf[0].item(), 4)
                warning_level = 0

                if cls in classes_to_be_detected and 10 < box_depth < 20 and confidence > 0.45:
                    print(f"{cls} detected at {box_depth:.2f} meters. Slow down.")
                    warning_level = 1
                elif cls in classes_to_be_detected and box_depth < 10 and confidence > 0.55:
                    safe_to_proceed = False
                    print(f"{cls} detected at {box_depth:.2f} meters. Applying brakes.")
                    warning_level = 2

                # Set color based on warning level
                color = (0, 255, 0)
                
                if warning_level == 1:
                    color = (0, 255, 255)
                elif warning_level == 2:
                    color = (0, 0, 255)

                # Draw rectangle on the frame
                thickness = 2
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, thickness)

                # Display class label, confidence, and depth
                cv2.putText(frame, f"{cls}: {confidence}, Depth = {box_depth:.2f}m", 
                            (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, thickness)
                
                safe_to_proceed = warning_level

        cv2.imshow(title, frame)

    if cv2.waitKey(1) == ord('q'):
        return False, safe_to_proceed

    return True, safe_to_proceed

def delete_actors(world):
    """
    Deletes actors from the world based on their type_id.

    Args:
        world: The world object from which actors will be deleted.

    Returns:
        None
    """
    for actor in world.get_actors():
        if actor.type_id.startswith("vehicle.") or actor.type_id.startswith("sensor.") or actor.type_id.startswith("walker."):
            actor.destroy()

def scenario_selection_ui():
    """
    Displays a GUI window for selecting a scenario for the experiment.

    Returns:
        str: The selected scenario value.
    """
    def select_scenario(selection):
        scenario_text = ""
        if selection == "1":
            scenario_text = "AEB with a Car"
        elif selection == "2":
            scenario_text = "AEB with a Bicycle"
        elif selection == "3":
            scenario_text = "Collision Warning"
        elif selection == "4":
            scenario_text = "Speed Bump Detection"
        elif selection == "5":
            scenario_text = "AEB with a Pedestrian"
        elif selection == "6":
            scenario_text = "Full Demo"
        else:
            scenario_text = "Exiting"
        messagebox.showinfo("Scenario Selected", f"Scenario {selection} selected: {scenario_text}")
        selected_scenario.set(selection)
        root.destroy()

    root = tk.Tk()
    root.title("Scenario Selection")

    # Set window size
    root.geometry("800x600")
    root.configure(bg="white")

    # Set font sizes and styles
    font = tkFont.Font(size=14)
    title_font = tkFont.Font(size=18, weight="bold")

    selected_scenario = tk.StringVar()

    # Add logos
    logo1_path = "Simulation(CARLA)/images/suzuki.png"
    logo2_path = "Simulation(CARLA)/images/tihan.png"
    logo1_img = tk.PhotoImage(file=logo1_path)
    logo2_img = tk.PhotoImage(file=logo2_path)
    tk.Label(root, image=logo1_img, bg="white").place(x=50, y=20, width=100, height=100)
    tk.Label(root, image=logo2_img, bg="white").place(x=650, y=20, width=100, height=100)

    # Title text
    tk.Label(root, text="Suzuki-TiHAN Final Year Demonstration", font=title_font, bg="white").pack(pady=10)
    tk.Label(root, text="28th August 2024", font=title_font, bg="white").pack()

    tk.Label(root, text="Select a Scenario for the Experiment:", font=font, bg="white").pack(pady=20)

    # Add spacing between buttons
    button_spacing = 10

    # Configure button style
    button_style = {
        "bg": "orange",
        "activebackground": "orange",
        "fg": "white",
        "bd": 2,
        "relief": "solid",
        "highlightthickness": 2
    }

    # Frame for buttons to ensure same size
    button_frame = tk.Frame(root, bg="white")
    button_frame.pack(pady=10)

    # Add buttons with modified style and ensure same size
    buttons = [
        ("AEB with a Car", "1"),
        ("AEB with a Bicycle", "2"),
        ("Collision Warning", "3"),
        ("Speed Bump Detection", "4"),
        ("AEB with a Pedestrian","5"),
        ("Full Demo", "6"),
        ("Exit", "exit")
    ]

    for text, value in buttons:
        if value == "exit":
            tk.Button(button_frame, text=text, command=root.quit, font=font, bg="red", fg="white", bd=2, relief="solid", highlightthickness=2).pack(fill=tk.X, pady=button_spacing, padx=20)
        else:
            tk.Button(button_frame, text=text, command=lambda v=value: select_scenario(v), font=font, **button_style).pack(fill=tk.X, pady=button_spacing, padx=20)
    root.mainloop()

    return selected_scenario.get()

def main():
    """
    The main function of the Suzuki demo script.
    
    This function sets up the CARLA client and world, spawns the vehicles and walkers, 
    initializes the ZED camera and YOLO models, and runs the simulation scenarios.
    It also handles the control of the vehicles and walkers, and displays the camera images.
    """
    scenario = scenario_selection_ui()
    try:
        scenario = int(scenario)
    except ValueError:
        print("Exiting")
        return

    # Create the CARLA client and world
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    traffic_manager = client.get_trafficmanager(8002) 

    delete_actors(world)

    blueprint_library = world.get_blueprint_library()
    vehicle_bp = blueprint_library.find('vehicle.mercedes.sprinter')

    spawn_points = world.get_map().get_spawn_points()
    yolo_model_path = "Simulation(CARLA)/models_checkpoint/collision_warning_yolov8.pt"
    yolo_model_sb_path = "Simulation(CARLA)/models_checkpoint/SB_PT.pt"

    test_vehicle = None
    walker = None

    defined_spawn_points = [0, 1, 2, 3, 4, 5, 6, 7]
    if scenario == 4:
        print("Demonstation of Speed Bump Detection and Pothole Detection")
        defined_spawn_points = [7,8,9,10,11,13]
        yolo_model_path = yolo_model_sb_path
        walker = not None
    elif scenario == 6:
        defined_spawn_points = [0,1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 13, 14]
        count_scenes = 0 

    defined_spawn_points = [spawn_points[i] for i in defined_spawn_points]

    vehicle = world.spawn_actor(vehicle_bp, defined_spawn_points[0])
    i = 1  # Index for spawn points

    count_scenes = 0

    print("Creating ZED Camera")
    zed_camera = ZEDCamera(world, blueprint_library, vehicle)

    print("Loading YOLO models")
    yolo_model = load_yolo_model(pretrained=False, path=yolo_model_path)
    yolo_model_sb = load_yolo_model(pretrained=False, path=yolo_model_sb_path)

    def destroy_vehicle(vehicle):
        try:
            vehicle.destroy()
            time.sleep(1)
        except:
            pass

    def destroy_walker(walker):
        try:
            walker.destroy()
            time.sleep(1)
        except:
            pass

    def create_test_vehicle(spawn_points, i, vehicle_type='car', autopilot=False):
        if vehicle_type == 'car':
            test_vehicle_bp = blueprint_library.find('vehicle.mini.cooper_s_2021')
        elif vehicle_type == 'bike':
            test_vehicle_bp = blueprint_library.find('vehicle.kawasaki.ninja')
        elif vehicle_type == 'bicycle':
            test_vehicle_bp = blueprint_library.find('vehicle.diamondback.century')
        test_vehicle = world.spawn_actor(test_vehicle_bp, spawn_points[i])
        test_vehicle.set_autopilot(autopilot,traffic_manager.get_port())
        return test_vehicle
    
    def create_test_walker(spawn_points, i, blueprint, autopilot = True ):
        walker_bp = blueprint_library.find(blueprint)
        walker = world.spawn_actor(walker_bp, spawn_points[i])
        if autopilot :
            walker_control = carla.WalkerControl()
            walker_control.speed = 2 # Set the walking speed (in meters per second)
            walker_control.direction = carla.Vector3D(1, 0, 0)  # Set the walking direction

            walker.apply_control(walker_control)
        return walker
    
    def scenario1():
        print("Demonstartion of Automatic Emergency Braking (AEB) with a Car")
        test_spawn_point = 17
        return create_test_vehicle(spawn_points, test_spawn_point, vehicle_type='car', autopilot=True)

    def scenario2():
        print("Demonstration of Automatic Emergency Braking (AEB) with a Bicycle")
        test_spawn_point = 17
        return create_test_vehicle(spawn_points, test_spawn_point, vehicle_type='bicycle', autopilot=True)

    def scenario3():
        print("Demonstration of Collision Warning")
        test_spawn_point = 7
        return create_test_vehicle(spawn_points, test_spawn_point, vehicle_type='car')

    def scenario5():
        print("Demonstration of AEB with a Pedestrian")
        test_spawn_point = 17
        return create_test_walker(spawn_points, test_spawn_point, blueprint = "walker.pedestrian.0034")
    
    def scenario6():
        print("Demonstration of Automatic Emergency Braking (AEB) and Collision Warning")
        test_spawn_point_1 = 17
        demo_vehicle_1 = create_test_vehicle(spawn_points, test_spawn_point_1, vehicle_type='car', autopilot=True)
        test_spawn_point_2 = 15
        walker_1 = create_test_walker(spawn_points, test_spawn_point_2, blueprint = "walker.pedestrian.0034", autopilot = False)
        return demo_vehicle_1, walker_1

    print("Initializing BasicAgent")
    agent = BasicAgent(vehicle, target_speed=15)
    
    # Ensure map is loaded
    time.sleep(5)

    print("Setting destination to the first target")
    try:
        agent.set_destination(defined_spawn_points[i].location)
    except Exception as e:
        print(f"Error setting destination: {e}")
        return

    time.sleep(2)  # Allowing sensors to initialize
    print("Press 'q' to quit.")

    while True:
        proceed, safe_to_proceed = process_and_display_images(zed_camera, yolo_model, title="Collision Warning Model")
        if scenario == 6:
            proceed_sb, _ = process_and_display_images(zed_camera, yolo_model_sb, title="Speed Bump Detection Model")

        if not proceed or (scenario == 6 and not proceed_sb):
            break

        control = agent.run_step()

        if safe_to_proceed == 2:
            count_scenes += 1
            control.throttle = 0
            control.brake = 1.0
            vehicle.apply_control(control)
            print("Demonstration of Automatic Emergency Braking")
            print("Removing Dummy") 
            time.sleep(0.5)
            if scenario == 6:
                if count_scenes == 1:
                    destroy_vehicle(test_vehicle_1)
                else:
                    destroy_walker(walker_1)
            elif scenario == 5:
                destroy_walker(walker)
            else:
                destroy_vehicle(test_vehicle)
            
            test_vehicle = not None
            continue
        elif safe_to_proceed == 1 :
            control.throttle = 0.75
            control.brake = 0.25
            vehicle.apply_control(control)

            print("Warning: Slow Down")
            continue

        vehicle.apply_control(control)

        if agent.done():
            print("The target has been reached, setting new target")
            if i + 1 == len(defined_spawn_points):
                print("All targets have been reached, exiting")
                break
            i += 1
            
            next_destination = defined_spawn_points[i]
            agent.set_destination(next_destination.location)

            if i == 2 and walker is None and scenario == 5:
                print("Creating a Pedestrian for AEB with a Pedestrian")
                walker = scenario5()
                test_vehicle = 1
                time.sleep(0.25)

            if i == 4 and test_vehicle is None:
                if scenario == 1:
                    print("Creating Dummy for AEB with a Car")
                    test_vehicle = scenario1()
                elif scenario == 2:
                    print("Creating Dummy for AEB with a Bicycle")
                    test_vehicle = scenario2()
                elif scenario == 3:
                    print("Creating Dummy for Collision Warning")
                    test_vehicle = scenario3()
                elif scenario == 6:
                    print("Creating Dummy for AEB and Collision Warning")
                    test_vehicle_1, walker_1 = scenario6()
                time.sleep(0.25)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    zed_camera.destroy()
    cv2.destroyAllWindows()
    delete_actors(world)

if __name__ == "__main__":
    main()
