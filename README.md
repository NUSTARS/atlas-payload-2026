IMPORTANT UPDATE 12/13/2025:

Repository has been renamed from "Roll-Control" and many big changes were made; please follow guide here:

1. run this command based on how you cloned from the repo; if your not sure, you probably used the HTTPS. This should change your remote URL, so if you try to do a push or pull, this should take care of any potential URL related issues. (Make sure you open your terminal to the directory containing this repo)

   SSH users: git remote set-url origin git@github.com:NUSTARS/atlas-payload-2026.git

   HTTPS: git remote set-url origin https://github.com/NUSTARS/atlas-payload-2026.git
   
3. rename your folder in local end; when you do a git pull, your container folder is likely still named "Roll-Control"; rename it to "atlas-payload-2026" to avoid any confusion with the folder within the repo called "Roll-Control"

4. if you have made any changes that would result in a merge conflict when pulling, or you have changes that may get overriden with a git pull, try the trouble shooting method below
   
if you have any other issues, feel free to message one of the electrical nerds or the big bosses (Max/Bernie)


Brute Force (slightly bullshit) Troubleshooting method:
1. pick another folder OUTSIDE of this github repo in your local machine
2. git clone this repository
3. add in your changes to the newly cloned repo
4. git add, commit, push
5. delete your old repo folder

example for my machine:

starting directory is 2026 -> Roll-Control

cloned repo to make 2026 folder contain -> Roll-Control, atlas-payload-2026

added changes to atlas-payload-2026

git add,commit,push

removed Roll-Control Folder from my local 

done!

