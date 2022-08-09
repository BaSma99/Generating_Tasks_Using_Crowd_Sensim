Generating_Tasks_Using_Crowd_Sensim

Consider this: 

    - We have two independent sub-assignments, 
    
          1) generating tasks, and 
          
          2) obtaining user movement event. 
          
          - Specifically, according to task requirements (provided in Table 1), generate MCS 2,000 tasks. 
          
          ![image](https://user-images.githubusercontent.com/54502733/183563587-c387b77f-4a0b-4cf0-9cb4-aba7c8bf7472.png)
          
          - Uniform mobility algorithm is applied in CrowdSenSim to model user movement and generate user movements events (in crowdsensim2.py).
          
          - Use stochastic algorithm and uniform algorithm to generate movements events separately, 
            under 3days, number of users = 15,000 (other configuration the same as default or configured as you needed)
