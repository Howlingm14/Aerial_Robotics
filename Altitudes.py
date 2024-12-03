import matplotlib.pyplot as plt
import numpy as np


ax = plt.gca()
ax.set_xlim([0, 10])
ax.set_ylim([6000, 6520])

fs=18

plt.xlabel("Ground Elevation [ft]", fontsize=fs)
plt.ylabel("Altitude AMSL [ft]", fontsize=fs)

# fig, ax = plt.subplots(figsize=(10, 10))
ax.set_xticks([1, 3, 5, 7, 9], labels=["5840", "5905", "5971", "6037", "6100"], fontsize=fs)
plt.yticks([6000, 6100, 6200, 6300, 6400, 6500], fontsize=fs)

plt.plot([0, 2], [6040, 6040], 'r-', label="Altitude Limits")
plt.plot([0, 2], [6240, 6240], 'r-')

plt.plot([2, 2], [6040, 6105], 'r--')
plt.plot([2, 2], [6240, 6305], 'r--')

plt.plot([2, 4], [6105, 6105], 'r-')
plt.plot([2, 4], [6305, 6305], 'r-')

plt.plot([4, 4], [6105, 6171], 'r--')
plt.plot([4, 4], [6305, 6371], 'r--')

plt.plot([4, 6], [6171, 6171], 'r-')
plt.plot([4, 6], [6371, 6371], 'r-')

plt.plot([6, 6], [6171, 6237], 'r--')
plt.plot([6, 6], [6371, 6437], 'r--')

plt.plot([6, 8], [6237, 6237], 'r-')
plt.plot([6, 8], [6437, 6437], 'r-')

plt.plot([8, 8], [6237, 6300], 'r--')
plt.plot([8, 8], [6437, 6500], 'r--')

plt.plot([8, 10], [6300, 6300], 'r-')
plt.plot([8, 10], [6500, 6500], 'r-')


plt.plot([0, 4], [6200, 6200], 'b--', label="Proposed Altitude")
plt.plot([4, 6], [6200, 6350], 'b--')
plt.plot([6, 10], [6350, 6350], 'b--')

plt.legend(loc="upper left", fontsize=16)
plt.show()