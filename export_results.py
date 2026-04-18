import csv
import os
from mip import solve_mip

instance_number_list = [1,2,3,4,5,6,7,8,9,10,11,12,13]
#instance_number_list = [10, 11, 12, 13]
num_runways_list = [1,2,3]
objectives = ['penalty', 'total_time', 'makespan']

output_file = 'results/optimization_results.csv'
fieldnames = ['instance_number', 'objective', 'number_of_runways', 'landing_times', 'time_diff_to_target', 'runway_assignment', 'objective_value', 'penalty_objective', 'total_time_objective', 'makespan_objective', 'time_taken', 'ub', 'lb', 'gap']

file_exists = os.path.exists(output_file)

with open(output_file, mode='a', newline='') as f:
    writer = csv.DictWriter(f, fieldnames=fieldnames)

    if not file_exists:
        writer.writeheader()

    for instance_number in instance_number_list:
        for num_runways in num_runways_list:
            for objective in objectives:
                
                print(f"Solving instance {instance_number} with {num_runways} runways and objective {objective}...")    
                landing_times, time_diff_to_target, runway_assignment, penalty_objective, total_time_objective, makespan_objective, time_taken, ub, lb, gap = solve_mip(instance_number, num_runways, objective, time_limit=10)

                writer.writerow({
                    'instance_number': instance_number,
                    'objective': objective,
                    'number_of_runways': num_runways,
                    'landing_times': landing_times,
                    'time_diff_to_target': time_diff_to_target,
                    'runway_assignment': runway_assignment,
                    'objective_value': ub,
                    'penalty_objective': penalty_objective,
                    'total_time_objective': total_time_objective,
                    'makespan_objective': makespan_objective,
                    'time_taken': time_taken,
                    'ub': ub,
                    'lb': lb,
                    'gap': gap
                })