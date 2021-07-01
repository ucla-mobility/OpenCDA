# -*- coding: utf-8 -*-
"""
This is the script that loop through the pdoc file and reformat it to fit the markdown file standard.
"""

import os 

def format_md_file(input_dir):
	"""
	Reformat the text with markfown file (mkdocs) standard.

        Args:
            -input_dir (string): The directory of the input text file.
        Returns:
            -lines_edit (list): The reformated text file stored in a list.
	"""
	
	with open(input_dir, 'r') as f:
	    # read line 
	    lines = f.readlines()
	f.close()

	lines_edit = []
	# edit file 
	for line in lines: 
	    # find not empty line
	    if len(line) > 1:
	        # start modify text

	        # identify module
	        if('Module' in line): 
	            line = '# '+ line[7:] # set module name to be the 1st level title 
	            lines_edit.append(line)

	        elif('====' in line): 
	            doc_index = lines.index(line) + 1
	            line = lines[doc_index]
	            lines_edit.append('Description for this module: ' + line + '\n')
	            continue

	        elif('----' in line): 
	        	lines.remove(line)
	        	continue

	        elif line[0]=='`': 

	            line = '## ' + line[1:-2] + '\n' # set 2nd level title for class name 
	            lines_edit.append(line)

	        elif line[0]==':': 
	        	line = line[4:]
	        	lines_edit.append(line + '\n')

	        elif(('Parameters' in line) or ('Attributes' in line)): 
	            line = '### ' + line[4:] # set 3rd level title for parameters and Attributes
	            lines_edit.append(line)

	        elif ('Methods' in line):
	            line = '### Methods ' + '\n'
	            lines_edit.append(line)

	        elif len(line) >= 4 and line[4] == '-': 
	            # find indexes
	            name_index = line.index(':')-1
	            description_index = line.index(':')+2
	            doc_index = lines.index(line)+1
	            # edit line
	            line = '- <font color="#f8805a">' + line[5:name_index] + '</font> (' + line[description_index:-1] + ')' + '\n'
	            lines_edit.append(line)
	            lines_edit.append(' ' + '\n')
	            # edit function doc
	            doc = lines[doc_index][8:]
	            lines_edit.append(doc + '\n')

	        # edit methods 
	        elif len(line)>=4 and line[4] == '`': 
	        	# find index 
	            name_end_index = line.index('(')
	            doc_index = lines.index(line)+1
	            # edit line
	            line = '- <font color="#7fb800">' + line[5:name_end_index] + '</font> ' + line[name_end_index:-2] + '\n'
	            lines_edit.append(line)
	            lines_edit.append(' ' + '\n')
	            # edit doc string
	            doc = lines[doc_index][8:]
	            lines_edit.append(doc + '\n')

	        # args and returns of the methods 
	        elif 'Args' in line:
	            line = '**Arguments:**' + '\n'
	            lines_edit.append(line + '\n')
	        elif 'Returns' in line:
	            line = '**Returns**' + '\n'
	            lines_edit.append(line + '\n')
	        # find actual return and arguments contents
	        elif len(line) >= 12 and line[12] == '-': 
	            break_index = line.index(':')+1
	            line = '*' + line[13:break_index] + '* ' + line[break_index:-1] + '\n' 
	            lines_edit.append(line + '\n')
	        # identify enumerate class
	        elif

	        else:
	        	continue
	    else: 
	    	continue

	return lines_edit

if __name__ == "__main__":
    tar_directory = '/home/xuhan/OpenCDA/pdocfile/opencda/'
    transform_docs = []
    for (root,dirs,files) in os.walk(tar_directory, topdown=True):
        for file in files: 
            # check if index
            if ('index' not in file) and ('version' not in file) and ('conf' not in file) and ('.md' in file):
                file_path = root + '/' + file 
                print(file_path)
                # transform md docs
                transform_docs = transform_docs + format_md_file(file_path)
                print('------------finish transform-------------')

    output_dir = '/home/xuhan/OpenCDA-doc/OpenCDA/docs/pythonapi_doc.md'
	# store it at target dir
    f = open( output_dir, 'a')
    f.writelines(transform_docs) 
    f.close()
        		