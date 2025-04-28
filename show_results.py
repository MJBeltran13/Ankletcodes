import pandas as pd
import os
import matplotlib.pyplot as plt
from matplotlib.image import imread

# Check if analysis results exist
if os.path.exists('analysis_results/summary_results.csv'):
    # Load and display summary results
    results = pd.read_csv('analysis_results/summary_results.csv')
    print("\n=== Jump and Speed Analysis Results ===")
    print(results)
    
    # Display key metrics
    if not results.empty:
        for i, row in results.iterrows():
            print(f"\nFile: {row['filename']}")
            print(f"Number of jumps detected: {row['num_jumps']}")
            print(f"Maximum vertical speed: {row['max_vertical_speed']:.2f} m/s")
            print(f"Maximum running speed: {row['max_running_speed']:.2f} km/h")
            print(f"Average running speed: {row['avg_running_speed']:.2f} km/h")
    
    # Display plots
    try:
        # Try to display the analysis plot
        print("\nAnalysis plots were saved to the 'analysis_results' directory")
        print("You can view them there for a detailed visualization of the data")
        
        # List all analysis images
        image_files = [f for f in os.listdir('analysis_results') if f.endswith('_analysis.png')]
        for img_file in image_files:
            print(f" - {img_file}")
        
    except Exception as e:
        print(f"Error displaying plots: {str(e)}")
else:
    print("No analysis results found. Please run process_ankle_data.py first.")

print("\nTo run the complete analysis:")
print("1. First collect data using Arduino: upload sensor_data_collector.ino to your Arduino Nano 33 BLE Sense")
print("2. Run data_collector.py to collect and save data from the Arduino")
print("3. Run process_ankle_data.py to analyze the collected data")
print("4. View the results in the 'analysis_results' directory") 