def read_pcd_file(filepath):
    with open(filepath, 'r') as file:
        lines = file.readlines()

    # Extract the header and point data
    header = []
    points = []
    read_header = True
    size_values = None
    type_values = None
    count_values = None

    for line in lines:
        if read_header:
            if line.startswith('FIELDS'):
                # Remove "intensity" from the fields
                fields = line.strip().split()
                fields = [field for field in fields if field != 'intensity']
                line = ' '.join(fields) + '\n'
            elif line.startswith('SIZE'):
                # Dynamically determine the number of elements and remove the final value
                size_values = line.strip().split()
                size_values.pop()  # Remove the final value
                line = ' '.join(size_values) + '\n'
            elif line.startswith('TYPE'):
                # Dynamically determine the number of elements and remove the final value
                type_values = line.strip().split()
                type_values.pop()  # Remove the final value
                line = ' '.join(type_values) + '\n'
            elif line.startswith('COUNT'):
                # Dynamically determine the number of elements and remove the final value
                count_values = line.strip().split()
                count_values.pop()  # Remove the final value
                line =' '.join(count_values) + '\n'
            header.append(line)
            if line.startswith('DATA'):
                read_header = False
        else:
            # Only keep the first three values (x, y, z)
            point_data = line.strip().split()[:3]
            if point_data:
                points.append(point_data)

    return header, points

def write_pcd_file(filepath, header, points):
    with open(filepath, 'w') as file:
        for line in header:
            file.write(line)
        for point in points:
            file.write(' '.join(point) + '\n')

def convert_pcd_axes(input_filepath, output_filepath):
    # Read the PCD file
    header, points = read_pcd_file(input_filepath)

    # Change the XYZ to ZXY for each point
    converted_points = []
    for point in points:
        if len(point) == 3:
            x, y, z = point
        elif len(point) == 4:
            x, y, z, _ = point
        # Reorder to ZXY (x to z, y to x, z to y)
        converted_points.append([z, x, y])

    # Write the updated PCD file
    write_pcd_file(output_filepath, header, converted_points)
    print(f"Converted PCD file saved to '{output_filepath}'")

# Use the function to convert a PCD file
input_pcd_file = '/tmp/finalCloud.pcd'  # Replace with the path to your input PCD file
output_pcd_file = '/tmp/finalCloud_converted.pcd'  # Replace with the desired path for your output PCD file
convert_pcd_axes(input_pcd_file, output_pcd_file)
