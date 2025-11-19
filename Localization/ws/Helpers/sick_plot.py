import matplotlib.pyplot as plt

# Given data
dist_samples = [2.75, 2.50, 2.00, 1.50, 1.00, 0.74, 0.585]  # x-axis: distance
adc_samples = [6370, 5821, 4706, 3600, 2481, 1721, 1638]  # y-axis: ADC values

# Plotting the points
plt.figure(figsize=(8, 5))
plt.scatter(dist_samples, adc_samples, color='blue', label='Data Points')
plt.title("Distance vs ADC Value")
plt.xlabel("Distance (inches)")
plt.ylabel("ADC Reading")
plt.grid(True)
plt.legend()
plt.gca().invert_xaxis()  # Optional: because ADC usually increases as distance decreases
plt.tight_layout()
plt.show()
