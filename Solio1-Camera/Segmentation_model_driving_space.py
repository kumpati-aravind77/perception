import torch
from PIL import Image
from torchvision import transforms
import numpy as np
import cv2
import time
import torch.nn as nn
import torch.nn.functional as F
from torchvision.models import resnet50

# Define the pyramid pooling module
class pyramid_pooling_module(nn.Module):
    def __init__(self, in_channels, out_channels, bin_sizes):
        super(pyramid_pooling_module, self).__init__()

        # Create pyramid pool layers
        self.pyramid_pool_layers = nn.ModuleList()
        for bin_sz in bin_sizes:
            self.pyramid_pool_layers.append(nn.Sequential(
                nn.AdaptiveAvgPool2d(bin_sz),
                nn.Conv2d(in_channels, out_channels, kernel_size=1, bias=False),
                nn.BatchNorm2d(out_channels),
                nn.ReLU(inplace=True)
            ))

    def forward(self, x):
        x_size = x.size()
        out = [x]
        for layer in self.pyramid_pool_layers:
            # Upsample pyramid pool layers and concatenate them
            out.append(F.interpolate(layer(x), x_size[2:], mode='bilinear', align_corners=True))
        return torch.cat(out, 1)

# Define the PSPNet model
class PSPNet(nn.Module):
    def __init__(self, in_channels, num_classes, use_aux=False):
        super(PSPNet, self).__init__()
        self.in_channels = in_channels
        self.num_classes = num_classes

        # Use resnet50 from torchvision without deprecated arguments
        # backbone = resnet50(pretrained=False)
        backbone = resnet50(weights=None)
        self.initial = nn.Sequential(*list(backbone.children())[:4])
        self.layer1 = backbone.layer1
        self.layer2 = backbone.layer2
        self.layer3 = backbone.layer3
        self.layer4 = backbone.layer4

        ppm_in_channels = int(backbone.fc.in_features)
        # Create pyramid pooling module and classifier
        self.ppm = pyramid_pooling_module(in_channels=ppm_in_channels, out_channels=512, bin_sizes=[1, 2, 3, 6])
        self.cls = nn.Sequential(
            nn.Conv2d(ppm_in_channels * 2, 512, kernel_size=3, padding=1, bias=False),
            nn.BatchNorm2d(512),
            nn.ReLU(inplace=True),
            nn.Dropout2d(p=0.1),
            nn.Conv2d(512, self.num_classes, kernel_size=1)
        )

        # Combine pyramid pooling module and classifier into the main branch
        self.main_branch = nn.Sequential(self.ppm, self.cls)

        self.use_aux = False
        if use_aux:
            self.use_aux = True
            # Modify the input channels for the aux branch
            self.aux_branch = nn.Sequential(
                nn.Conv2d(512, 256, kernel_size=3, padding=1, bias=False),
                nn.BatchNorm2d(256),
                nn.ReLU(inplace=True),
                nn.Dropout2d(p=0.1),
                nn.Conv2d(256, self.num_classes, kernel_size=1)
            )

    def forward(self, x):
        input_size = x.shape[-2:]

        # Pass input through the layers of ResNet
        x = self.initial(x)
        x = self.layer1(x)
        x_aux = self.layer2(x)
        x = self.layer3(x_aux)
        x = self.layer4(x)

        # Pass through the main branch
        main_output = self.main_branch(x)
        main_output = F.interpolate(main_output, size=input_size, mode='bilinear', align_corners=True)

        if self.use_aux:
            # Pass through the aux branch
            aux_output = F.interpolate(self.aux_branch(x_aux), size=input_size, mode='bilinear', align_corners=True)
            output = {'aux': aux_output, 'main': main_output}
            return output

        return main_output


# Define the transformation for the input images during testing
test_transform = transforms.Compose([
    transforms.Resize((360, 640)),
    transforms.ToTensor(),
    transforms.Normalize(mean=(0.485, 0.456, 0.406), std=(0.229, 0.224, 0.225)),
])

# Load the trained model
model = PSPNet(in_channels=3, num_classes=3, use_aux=True)
model.load_state_dict(torch.load('/home/tihan/Desktop/DrivingSpace/PSPNet_trained_weights.pth'))
model.eval()

# Input video path
#input_video_path = '/home/suzuki/Downloads/docker-aravind/segmentation-gpu-main/test_bed008.mp4'

# Output video path
output_video_path = '/home/tihan/Desktop/DrivingSpace/outcome/2.mp4'

# Open the input video file
cap = cv2.VideoCapture(0)

# Set the desired fps
desired_fps = 20
frame_time = 1.0 / desired_fps

# Get the video properties
width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

# Create VideoWriter object for output streaming
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
out = cv2.VideoWriter(output_video_path, fourcc, desired_fps, (width, height))

while True:
    start_time = time.time()

    # Read a frame from the input video
    ret, frame = cap.read()

    if not ret:
        break

    # Convert the frame to RGB
    input_image = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))

    # Apply the transformation
    input_image_tensor = test_transform(input_image).unsqueeze(0)

    # Perform inference
    with torch.no_grad():
        output = model(input_image_tensor)

    # Post-process the output
    segmentation_map = output['main'].argmax(dim=1).squeeze().cpu().numpy()

    # Define color mappings
    color_map = {
        0: [0, 0, 255],   # Non-drivable area (Red)
        1: [0, 255, 0],   # Drivable area (Green)
    }

    # Convert segmentation map to colored image
    colored_segmentation_map = np.zeros((360, 640, 3), dtype=np.uint8)
    for class_index, color in color_map.items():
        colored_segmentation_map[segmentation_map == class_index] = color

    # Resize segmentation map to match frame dimensions
    colored_segmentation_map_resized = cv2.resize(colored_segmentation_map, (width, height))

    # Combine original frame with resized colored segmentation map
    overlay = cv2.addWeighted(frame, 0.7, colored_segmentation_map_resized, 0.3, 0)

    # Write the overlayed frame to the output video
    out.write(overlay)

    # Display the result (optional, comment this out if you don't want to display)
    cv2.imshow('Overlayed Frame', overlay)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    # Sleep to ensure the loop runs at the desired frame rate
    elapsed_time = time.time() - start_time
    sleep_time = max(0, frame_time - elapsed_time)
    time.sleep(sleep_time)

# Release resources
cap.release()
out.release()
cv2.destroyAllWindows()
