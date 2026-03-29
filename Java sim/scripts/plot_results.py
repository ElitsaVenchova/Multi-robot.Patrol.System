import pandas as pd
import matplotlib.pyplot as plt

# Load CSV
file_path = "../outputs/results/aggregated_results.csv"
df = pd.read_csv(file_path)

# Group by strategy
strategies = df["strategy"].unique()

# =========================
# 1️⃣ Success Rate Plot
# =========================
plt.figure()

for strategy in strategies:
    subset = df[df["strategy"] == strategy]
    subset = subset.sort_values("lambda")

    plt.errorbar(
        subset["lambda"],
        subset["mean_success"],
        yerr=subset["std_success"],
        marker='o',
        label=strategy
    )

plt.xlabel("Lambda (Attack Rate)")
plt.ylabel("Mean Success Rate")
plt.title("Success Rate vs Attack Intensity")
plt.legend()
plt.grid()

plt.savefig("../outputs/plots/success_rate_plot.png")

# plt.savefig("figure.pdf")

plt.close()

# =========================
# 2️⃣ Detection Time Plot
# =========================
plt.figure()

for strategy in strategies:
    subset = df[df["strategy"] == strategy]

    plt.errorbar(
        subset["lambda"],
        subset["mean_detection"],
        yerr=subset["std_detection"],
        marker='o',
        label=strategy
    )

plt.xlabel("Lambda (Attack Rate)")
plt.ylabel("Average Detection Time")
plt.title("Detection Time vs Attack Intensity")
plt.legend()
plt.grid()

plt.savefig("../outputs/plots/detection_time_plot.png")
plt.close()

print("Plots generated successfully.")