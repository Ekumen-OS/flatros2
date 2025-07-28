import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

# Read the CSV, skipping non-data lines
with open('latency_log.csv', 'r') as f:
    lines = f.readlines()

# Find the header line
for i, line in enumerate(lines):
    if line.strip().startswith('size_bytes'):
        header_idx = i
        break
else:
    raise ValueError('No header found in latency_log.csv')

# Read the data into a DataFrame
from io import StringIO
data_str = ''.join(lines[header_idx:])
df = pd.read_csv(StringIO(data_str))

# Plot
plt.figure(figsize=(10,6))
plt.plot(df['size_bytes'], df['latency_ms'], marker='.', linestyle='-', alpha=0.7)
plt.xlabel('Message Size (bytes)')
plt.ylabel('Latency (ms)')
plt.title('Message Latency vs. Size')
plt.grid(True)
plt.tight_layout()
plt.savefig('latency_vs_size.png')
plt.show()

# Print summary statistics
print('Latency statistics by message size range:')
for start in range(0, df['size_bytes'].max(), 100000):
    end = start + 100000
    subset = df[(df['size_bytes'] >= start) & (df['size_bytes'] < end)]
    if not subset.empty:
        print(f"{start} - {end} bytes: count={len(subset)}, mean={subset['latency_ms'].mean():.3f} ms, min={subset['latency_ms'].min():.3f} ms, max={subset['latency_ms'].max():.3f} ms")
