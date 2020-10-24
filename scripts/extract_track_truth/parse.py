import xml.etree.ElementTree as ET

tree = ET.parse('model.sdf')
root = tree.getroot()

prefix = 'model://'

output = {}

for child in root[0]:
    pose = child.find('pose')

    if pose is None:
        continue

    uri = child.find('uri').text

    if uri not in output:
        output[uri] = []
        output[uri].append(pose)
    else:
        output[uri].append(pose)

for key in output.keys():
    with open("{}.csv".format(key[len(prefix):]), 'w') as outfile:
        for pose in output[key]:
            outfile.write(", ".join(pose.text.split(' ')))
            outfile.write('\n')
