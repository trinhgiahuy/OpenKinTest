!#/bin/bash
# A bash script to check the executables for all packages 
rospack list-names | while read pkgname; do
    echo "Executables for package '${pkgname}':";
    rospack-list-executables $pkgname; echo "";
done

# Enable package autoccompletion for your newly created command
# $ complete -F _roscomplate rospack-list-executables
#
# Apped to to .bashrc for everytime boot up
# $ echo "complete -F _roscomplate rospack-list-executables" 2> ~/.bashrc
