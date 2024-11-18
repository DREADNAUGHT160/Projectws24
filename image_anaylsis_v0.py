import cv2
import numpy as np
from robotpy_apriltag import AprilTagDetector
from skimage.measure import shannon_entropy

# Initialize the detector
detector = AprilTagDetector()
detector.addFamily("tag36h11")  # Add a commonly used tag family

def load_image(filepath):
    image = cv2.imread(filepath, cv2.IMREAD_GRAYSCALE)
    if image is None:
        print(f"Error: Could not load image at {filepath}.")
        exit()
    return image

def compute_sharpness(image):
    laplacian_var = cv2.Laplacian(image, cv2.CV_64F).var()
    return laplacian_var

def compute_contrast(image):
    contrast = image.max() - image.min()
    return contrast

def compute_colorfulness(image):
    image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)  # Convert to color for colorfulness computation
    rg = image[..., 0] - image[..., 1]
    yb = 0.5 * (image[..., 0] + image[..., 1]) - image[..., 2]
    std_root = np.sqrt(np.mean(rg ** 2) + np.mean(yb ** 2))
    mean_root = np.mean(np.abs(rg)) + np.mean(np.abs(yb))
    return std_root + 0.3 * mean_root

def compute_entropy(image):
    entropy = shannon_entropy(image)
    return entropy

def compute_brightness(image):
    brightness = np.mean(image)
    return brightness

def main():
    # Load the image
    filepath = 'AprilhTags.png'  # Replace with your image file
    image = load_image(filepath)

    # Detect tags
    detections = detector.detect(image)
    image_color = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)  # Convert to color for visualization
    print(f"Detected {len(detections)} tags")

    # Extract detected tag IDs
    detected_ids = [detection.getId() for detection in detections]

    # Group and target logic
    all_tags = list(range(48))  # Represents all possible tags from 0 to 47
    group_size = 12
    groups = [all_tags[i:i + group_size] for i in range(0, len(all_tags), group_size)]
    targets = {i + 1: group for i, group in enumerate(groups)}
    detected_counts = {target_number: sum(1 for tag_id in detected_ids if tag_id in group) for target_number, group in targets.items()}

    # Display groups and detected counts
    for target_number, group in targets.items():
        print(f"Target {target_number} group contains {len(group)} tags: {group}")
        print(f"Target {target_number} detected {detected_counts[target_number]} tags")

    # Mark detected tags on the image
    for detection in detections:
        tag_id = detection.getId()
        center = detection.getCenter()
        cv2.circle(image_color, (int(center.x), int(center.y)), radius=5, color=(0, 255, 0), thickness=-1)
        cv2.putText(image_color, f"ID: {tag_id}", (int(center.x) - 10, int(center.y) - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

    # Compute image quality metrics
    sharpness = compute_sharpness(image)
    contrast = compute_contrast(image)
    colorfulness = compute_colorfulness(image)
    entropy = compute_entropy(image)
    brightness = compute_brightness(image)

    # Display results of metrics
    print(f"Sharpness (Laplacian Variance): {sharpness}")
    print(f"Contrast: {contrast}")
    print(f"Colorfulness: {colorfulness}")
    print(f"Entropy: {entropy}")
    print(f"Brightness: {brightness}")

    # Write metrics on the image
    metrics_text = [
        f"Sharpness: {sharpness:.2f}",
        f"Contrast: {contrast:.2f}",
        f"Colorfulness: {colorfulness:.2f}",
        f"Entropy: {entropy:.2f}",
        f"Brightness: {brightness:.2f}"
    ]

    for i, text in enumerate(metrics_text):
        cv2.putText(image_color, text, (10, 30 + i * 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    # Save the marked image
    output_filepath = 'output_with_tags_and_metrics.png'
    cv2.imwrite(output_filepath, image_color)
    print(f"Output image saved as {output_filepath}")

if __name__ == "__main__":
    main()
