# pole-placement script

This script allow you to assign pole by standard control form. Then you system must be reachable!
It's for education purpose only! 

Problem solved by this script: https://en.wikipedia.org/wiki/Full_state_feedback
Algorithm use controllable canonical form. 
Steps:
  * Transform your system into standard form
      * You have A and B. 
      * Check reachable
      * Compute eigenvalues and their coefficients
      * Build Ac (A in standard form)
      * Build Bc (it's trivial)
      
  * Find Kc that control you system
      * We just use the definition of pole placement theorem
      
  * Tansform Kc in K
      * To transform Kc into K, according with theory, just compute: K = Kc * inv(R * inv(Rc))
  * Done


Of course, already exist a usefull command on matlab: https://www.mathworks.com/help/control/ref/place.html
Always on matlab exist a very powerful tool that help you in pole placement: https://www.mathworks.com/help/control/ref/controlsystemdesigner-app.html
With sisotool you can also never need to know what you doing, sisotool know the theory then do it all for you.


Yet, this script can show you all the calculations made to find the solution. Then it can be usefull to check if your calculation are correct or if there has been a mistake.


Requiered:
  * Octave
  * symbolic package

How to satisfy the requirements:
  * Open your terminal
  * $ sudo apt install octave
  * $ sudo apt install octave-symbolic
  * $ git clone https://github.com/stefano5/pole-placement-octave.git
  
Then open octave (within/without GUI) then give it:
  * addpath("/home/YOUR_USER_NAME/pole-placement-octave", 1)
  * savepath()
  
  Now you can use this script.
