import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import os

def view_sensor_data():
    # Path to the analysis results directory
    analysis_dir = "analysis_results"
    
    # Check if directory exists
    if not os.path.exists(analysis_dir):
        print(f"Error: Directory '{analysis_dir}' not found")
        return
    
    # List all PNG files in the directory
    png_files = [f for f in os.listdir(analysis_dir) if f.endswith('.png')]
    
    if not png_files:
        print("No analysis images found in the directory")
        return
    
    # Display each image
    for img_file in png_files:
        img_path = os.path.join(analysis_dir, img_file)
        img = mpimg.imread(img_path)
        
        plt.figure(figsize=(12, 8))
        plt.imshow(img)
        plt.axis('off')  # Hide axes
        plt.title(f'Sensor Data Analysis: {img_file}')
        plt.show()

if __name__ == "__main__":
    view_sensor_data() 