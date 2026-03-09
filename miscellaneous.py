#Read the instances files

def read_instance(instance_number):
    instance_file = f"instances/airland{instance_number}.txt"
    with open(instance_file, 'r') as file:
        lines = file.readlines()
    
    # Parse all numbers into a single vector
    data = []
    for line in lines:
        data.extend(map(float, line.split()))
    
    # First two values: number of planes and freeze time
    p = int(data[0])
    freeze_time = int(data[1])
    
    planes = []
    idx = 2  # Start after p and freeze_time

    appearance_time_vector = []
    earliest_landing_vector = []
    target_landing_vector = []
    latest_landing_vector = []
    penalty_early_vector = []
    penalty_late_vector = []
    separation_times_matrix = []
    
    for _ in range(p):
        # Read plane data (6 values)
        appearance_time = data[idx]
        earliest_landing = data[idx + 1]
        target_landing = data[idx + 2]
        latest_landing = data[idx + 3]
        penalty_early = data[idx + 4]
        penalty_late = data[idx + 5]
        idx += 6

        # Append to respective vectors
        appearance_time_vector.append(appearance_time)
        earliest_landing_vector.append(earliest_landing)
        target_landing_vector.append(target_landing)
        latest_landing_vector.append(latest_landing)
        penalty_early_vector.append(penalty_early)
        penalty_late_vector.append(penalty_late)
        
        # Read separation times (p values)
        separation_times = data[idx:idx + p]
        idx += p

        separation_times_matrix.append(separation_times)
        
        planes.append({
            'appearance_time': appearance_time,
            'earliest_landing': earliest_landing,
            'target_landing': target_landing,
            'latest_landing': latest_landing,
            'penalty_early': penalty_early,
            'penalty_late': penalty_late,
            'separation_times': separation_times
        })

    instance = {
        'number_of_planes': p,
        'freeze_time': freeze_time,
        'planes': planes,
        'appearance_time_vector': appearance_time_vector,
        'earliest_landing_vector': earliest_landing_vector,
        'target_landing_vector': target_landing_vector,
        'latest_landing_vector': latest_landing_vector,
        'penalty_early_vector': penalty_early_vector,
        'penalty_late_vector': penalty_late_vector,
        'separation_times_matrix': separation_times_matrix
    }
    
    return instance

'''
#Example usage:
instance_number = 13  # Change this to read different instances
instance = read_instance(instance_number)
print(f"Number of planes: {instance['number_of_planes']}")
print(f"Freeze time: {instance['freeze_time']}")
for i, plane in enumerate(instance['planes']):
    print(f"Plane {i + 1}: {plane}")
'''