# pole-placement script

This script allow you to assign pole by standard control form. Then your system must be stable.
It is for education purpose only! 

This problem solved by this script is here described: https://en.wikipedia.org/wiki/Full_state_feedback
Algorithm use controllable canonical form. 
Steps:
  * Transform your system into standard form
      * You have A and B matrix
      * Check if it is reachable
      * Compute eigenvalues and their coefficients
      * Build Ac (A in standard form)
      * Build Bc
      
  * Find Kc able to control your system
      * We just use the definition of the pole placement theorem
  
  * Tansform Kc of [Ac, Bc, Cc, 0] system for the original system [A, B, C, 0]
      * To transform Kc into K, according with theory, it is needed just compute: K = Kc * inv(R * inv(Rc))
  * Done


Of course, already exist a usefull command on matlab: https://www.mathworks.com/help/control/ref/place.html
Always on matlab exist a very powerful tool that help you in pole placement: https://www.mathworks.com/help/control/ref/controlsystemdesigner-app.html
With sisotool you do not need to know what you are doing, sisotool know the theory


But this script can show you all the calculations made to find the solution. Note that in this problem the solution, if exist, is unique. Then it can be usefull to check if your calculation are correct or if there has been a mistake.


Requiered:
  * Octave
  * symbolic package

How to satisfy the requirements:
  * Open your terminal (in linux based system), then:
  * $ sudo apt update
  * $ sudo apt install octave
  * $ sudo apt install octave-symbolic
  * $ git clone https://github.com/stefano5/pole-placement-octave.git
  
Then open octave (with/without GUI) then give it:
  * addpath("/home/YOUR_USER_NAME/pole-placement-octave", 1)
  * savepath()
  
  Now you can use this script
