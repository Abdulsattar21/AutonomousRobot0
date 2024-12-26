a_list = ["abc", "def", "ghi", "hello", "this is", "me"]
textfile = open("a_file.txt", "w" )
for element in a_list:
    textfile.write(element + "\n")
textfile.close()

txt_file = open("a_file.txt", "r")
file_content = txt_file.read()
print("The file content are: ", file_content)
print(file_content)
content_list = file_content.split(",")
print(content_list)
txt_file.close()
print("The list is: ", content_list)
for e in range(len(content_list)):
    print(content_list[e])