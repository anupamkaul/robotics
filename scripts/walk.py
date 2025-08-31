'''
upload scans all the github links of all folders in the 
path provided (or ./ by default). Collects and 
uploads all of these to a file. (useful to replicate
my other_git codebases)

download does the opposite and collects all of the structure
locally, including folder names

method : this should be a tree search (either breadth first or
depth first) where a leaf node is one that yields a valid
string for git remote -v

to make this simpler:

1. We let os.walk do the traversal for us in depth first search

2. Only when we see a .git in the current folder, we do a git remote -v and capture it
(root_folder (relative) + dirpath and cmd_output as JSON fields so they can be
re-used for reconstructing) 

3. Technically (2) is a leaf. If we find a way to prune the processing in os.walk
we do it, else we leave it and let it continue (there could be corner cases of 
3rd party git installs under a .git (with a different .git) that a prune would
miss capturing).
'''

import os
import subprocess

def scan_all_folders(root_folder):
    """
    Scans a given root folder and yields the full paths of all subfolders.

    Args:
        root_folder (str): The path to the folder to start scanning from.

    Yields:
        str: The full path of each subfolder found.
    """

    BOLD  = '\033[1m'
    RESET = '\033[0m'
    BLUE  = '\033[94m'

    command = ['git', 'remote', '-v']
    #command = ['ls', '-l']

    rootpath = os.getcwd()
    #print("root path: ", rootpath) 

    f = open("gitpaths.txt", "a") 

    for dirpath, dirnames, filenames in os.walk(root_folder):

        # dirpath is the current directory being walked
        # dirnames is a list of subdirectories in dirpath
        # filenames is a list of files in dirpath

        #print("\nsingle loop")
        #print("dirpath: ", dirpath)
        #print("dirnames: ", dirnames)
        #print("\n")
        #input()
        #continue # revelation : the os.walk construct itself is a depth first search

        os.chdir(rootpath)
        os.chdir(dirpath)

        #print("Evaluating current dir (chdir'd): ", os.getcwd(), "\n")

        '''
        # template:
        # do stuff..
        # go back
        os.chdir(rootpath)
        #print("do stuff and go back to ", os.getcwd())
        continue 
        '''

        if (os.path.exists(".git")):

            # execute the command and capture its output
            #print("found .git, getting git remote -v info now")
            #input()

            try:

                # you actually need to chdir into the dir, else this is executing at root level always ! 
                result = subprocess.run(command, capture_output=True, text=True, check=True)

                #print("For " , dirpath, "we have: ",result.stdout)
                print(f"For {BOLD}{dirpath} {RESET} we have: {BOLD}{BLUE}{result.stdout}{RESET}")
                json_txt = " { " + dirpath + " : " + result.stdout + " } "
                f.write(json_txt)

                if result.stderr:
                    print("error: ", result.stderr)

                # this is a leaf condition (a git remote exists, don't need to go down any further as all
                # artifacts below would report the same git remote data). We PRUNE this leaf out to stop
                # further descents into search, while still relying on python's os.walk structure

                '''
                print("remove ", dirpath) 
                print("first: ", dirnames, "\n")
                #dirnames.remove(dirpath)
                print("now: ", dirnames, "\n")
                '''

            except subprocess.CalledProcessError as e:

                # this is unlikely and needs investigations
                print(f"Error executing command: {e}")
                print(f"Stderr: {e.stderr}")
                print(e.returncode, e.output)

            except FileNotFoundError:
                print(f"Error: Command '{command[0]}' not found.")

                #print("on error go back to parent folder")
                os.chdir("../")
                #print("current dir: ", os.getcwd())
                #input()

        # Yield the current directory itself, if it's a subfolder of the root
        # (excluding the root_folder itself if only subfolders are desired)

        '''
        if dirpath != root_folder:
            yield dirpath
        '''

        # Optionally, you can also process the subdirectories found in dirnames
        # for dirname in dirnames:
        #     full_subdir_path = os.path.join(dirpath, dirname)
        #     # You could do something with full_subdir_path here if needed
        #     # The os.walk will automatically traverse these subdirectories
        #     # in subsequent iterations.

        
        #print("normal: go back to parent folder")
        os.chdir(rootpath)
        #print("current dir: ", os.getcwd())
        #input()

    f.close()

# Example usage:
if __name__ == "__main__":
    target_folder = "."  # Replace with the path to your desired folder
    #target_folder = "./dolma"  # Replace with the path to your desired folder
    print(f"Scanning folders in: {os.path.abspath(target_folder)}")

    for folder_path in scan_all_folders(target_folder):
        print(folder_path)
