#!/usr/bin/env python3

import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import argparse


# Parse arguments
parser = argparse.ArgumentParser(description="Analyze latency log CSV and plot results.")
parser.add_argument('-p', '--path', type=str, default='latency_log.csv', help='Path to latency log CSV file (default: latency_log.csv)')
args = parser.parse_args()

# Read the CSV, skipping non-data lines
with open(args.path, 'r') as f:
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
# Ensure size_bytes is numeric
df['size_bytes'] = pd.to_numeric(df['size_bytes'], errors='coerce')
df = df.dropna(subset=['size_bytes'])
df['size_bytes'] = df['size_bytes'].astype(int)


# Convert size to kilobytes for plotting and stats
df['size_kb'] = df['size_bytes'] / 1024

# Plot
plt.figure(figsize=(10,6))
plt.semilogx(df['size_kb'], df['latency_ms'], marker='.', linestyle='-', alpha=0.7)
plt.xlabel('Message Size (kB)')
plt.ylabel('Latency (ms)')
plt.title('Message Latency vs. Size')
plt.grid(True)
plt.tight_layout()
plt.savefig('latency_vs_size.png')
plt.show()

# # Print summary statistics
# print('Latency statistics by message size range (kB):')
# step_kb = 100
# for start_kb in range(0, int(df['size_kb'].max()) + step_kb, step_kb):
#     end_kb = start_kb + step_kb
#     subset = df[(df['size_kb'] >= start_kb) & (df['size_kb'] < end_kb)]
#     if not subset.empty:
#         print(f"{start_kb} - {end_kb} kB: count={len(subset)}, mean={subset['latency_ms'].mean():.3f} ms, min={subset['latency_ms'].min():.3f} ms, max={subset['latency_ms'].max():.3f} ms")

