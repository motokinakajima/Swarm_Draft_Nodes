import numpy as np
import scipy.io
import matplotlib.pyplot as plt

min_lon, max_lon = -72.251444, -72.241181
min_lat, max_lat = 43.741064, 43.743681

lon_line = np.linspace(min_lon, max_lon, 50)
lat_line = np.linspace(min_lat, max_lat, 50)
lonMesh, latMesh = np.meshgrid(lon_line, lat_line)

num_gaussians = 50
np.random.seed(42)

gaussians = []
for i in range(num_gaussians):
    center_x = np.random.uniform(-50.0, 50.0)
    center_y = np.random.uniform(-50.0, 50.0)
    
    amplitude = np.random.uniform(-100.0, 100.0)
    sigma = np.random.uniform(5.0, 25.0)
    
    gaussians.append({'cx': center_x, 'cy': center_y, 'amp': amplitude, 'sig': sigma})

zMean = np.zeros(lonMesh.shape)

for r in range(50):
    for c in range(50):
        cpp_x = -50.0 + (c / 49.0) * 100.0
        cpp_y = -50.0 + (r / 49.0) * 100.0
        
        scalar_val = 0.0
        for g in gaussians:
            dx = cpp_x - g['cx']
            dy = cpp_y - g['cy']
            dist_sq = dx * dx + dy * dy
            
            gaussian_val = g['amp'] * np.exp(-dist_sq / (2.0 * g['sig'] * g['sig']))
            scalar_val += gaussian_val
            
        zMean[r, c] = scalar_val

zVar = np.abs(np.random.normal(0.5, 0.1, lonMesh.shape))

data_package = {
    'latMesh': latMesh,
    'lonMesh': lonMesh,
    'zMean': zMean,
    'zVar': zVar
}
scipy.io.savemat('wilsons_landing_multimodal.mat', data_package)
print("Successfully generated 'wilsons_landing_multimodal.mat' using C++ logic!")

plt.figure(figsize=(10, 6))
mesh = plt.pcolormesh(lonMesh, latMesh, zMean, shading='auto', cmap='coolwarm')
plt.colorbar(mesh, label='Summed Multi-Modal Scalar Value')
plt.xlabel('Longitude')
plt.ylabel('Latitude')
plt.title("Wilson's Landing: Multi-Modal Gaussian Node Simulation")
plt.grid(True, linestyle='--', alpha=0.3)
plt.show()
