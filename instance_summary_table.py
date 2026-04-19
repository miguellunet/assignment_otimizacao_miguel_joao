import os
from miscellaneous import read_instance
from tabulate import tabulate

# List all instance files
instance_folder = 'instances'
instance_files = sorted([f for f in os.listdir(instance_folder) if f.startswith('airland') and f.endswith('.txt')])

summary = []

for fname in instance_files:
    instance_number = int(fname.replace('airland', '').replace('.txt', ''))
    instance = read_instance(instance_number)

    summary.append([
        int(instance_number),
        int(instance['number_of_planes']),
        int(min(instance['appearance_time_vector'])),
        int(max(instance['appearance_time_vector'])),
        int(min(instance['earliest_landing_vector'])),
        int(max(instance['latest_landing_vector'])),
        min(instance['penalty_early_vector']),
        max(instance['penalty_late_vector'])
    ])

headers = [
    'Instance', 'Planes', 'Earliest Appearance', 'Latest Appearance',
    'Earliest Landing', 'Latest Landing', 'Min Early Penalty', 'Max Late Penalty'
]

latex_table = tabulate(sorted(summary), headers, tablefmt='latex_booktabs', floatfmt='.2f')
print(latex_table)
