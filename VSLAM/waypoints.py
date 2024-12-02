import yaml


file_path = '3db.txt'
with open(file_path, 'r') as file:
    yaml_data = list(yaml.safe_load_all(file))


x_values = []
y_values = []
for data in yaml_data:
    if data is not None and 'header' in data and 'seq' in data['header'] and 'pose' in data and 'pose' in data['pose'] and 'position' in data['pose']['pose']:
        seq = data['header']['seq']
        x = data['pose']['pose']['position']['x']
        y = data['pose']['pose']['position']['y']
        x_values.append(x)
        y_values.append(y)
    else:
        print("Warning: Skipped a document with missing or incorrect structure.")


output_file = 'lastwp.txt'  
with open(output_file, 'w') as out_file:
    for x, y in zip(x_values, y_values):
        out_file.write(f"[{x},{y}],\n")

print("Output has been saved to:", output_file)