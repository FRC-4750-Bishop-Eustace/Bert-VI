//package org.usfirst.frc.team4750.robot;                                                                                                                                                                                         
//                                                                                                        ``....----------......```                                                                       
//                                                                                        `..-:::/++ooossssssssssooooooooooossssssssooo++//::-.``                                                         
//                                                                               `..-:/+osssssooo+++///:::------------------------::::///++oooossoo+/:-.`                                                 
//                                                                         `.:/+ossssoo++//:----------------------------------------------------:://++ooso+/-.`                                           
//                                                                   `-:/+ossso++/::--------------------------------------------------------------------::/++ooo+:.                                       
//                                                              `-:+ossso+//:---------------------------------------------------------------------------------::/+:-.``                                   
//                                                          `.:+osso+/::---------------------------------------------------------------------....-----------------------..`                               
//                                                      `-:+soo+/:------------------------------------------------------------------.````` ` `..``.----------------.......``                              
//                                                `.:/+ooso+/:--------------------------------------------.......`````..------------` -ooo` -syy- `........```````` `..  `-:/+:.                          
//                                             .:+ossso+/:---------------------------------..------------` `...--:::::. `.----...--.` /hho  -oo:`  ``.---` `/+o/   `so..+s/--yhy. `.`                     
//                                         `.:ossso+/:----------------------------------.``````.`````.---` -yyhs+++yhhy: `..```````` `yhh. .+oo.  .+o+/shy: ohho  `os./yh:`.:hys` `--.`                   
//                                      `-/ossso/:---------...`````..------------------. `/ss. `-//  `.-.` /hhh-  `ohhy.   .:+oss+.  /hho  /hho `+yy.``+hh+ :hhs``oo`/hhy+o+/-.` `.----..`                
//                                   `-+oso//:.```.-----.````.-:/:-  .-------........``` `oho``ohh+.` `-. `yhhs..:oys/`  .oy+.`:hho `yhh- `yhh.`ohh+/+o+:.` .hhh-o+` shhy-`  .-  .--------.`              
//                                `.+osso/` .--:-  .--.` .:osoo+os+  `````..````.` .-::. ./:`+shhyo+- `.` /hhhyssyhyo:` -yho.-/sso: :hho  /hh+ -hhho-.` ``  `yhhs+`  :yhhsoo++- `.---------..`            
//                             `-/+ssso/:.  +hhh: `-.` .+yy/.```````-:://:.`  :yys+oyhhs```  .shh/````.. `yhhs.``.shhh-`yhhs+/-.`  `yhh- `yhh. .yhhs:://+`   +ys/``.` `-::-.````.------------.`           
//                           `/ossso/:---. `shho` ..` :yhy- `..-.  -+/:/yhh:  ohh/.`/hho  .` -hhs` .---` :hhh:   .shhs`.hhho.``./- /hhs  /yys   ./+++/:.``````````.--..```....-----------------.`         
//                        `-/osss+/------` -hhh: `.` /hhh: `.---.`  `-:+yhh- .hhy` `shh- `.  shh:  .--. `shhy--:oyhs/`  :syyso+/-` :/:-  ...` `.```````...--.....-------------------------------.`        
//                      ./ossso+:-------. `shhs` .` .yhhy` .---.``:os+:/hhs  ohh/  :hho  .` .hhh/-` ..` :yyysooo+:-```.` `...``````````````` `.---------------------------------------------------`       
//                    ./ossso/:---------` :hhh: `.` /hhhs  ..`` `ohy- `shh: .hhy` `shh: `-` .osso/` ..` `..```      `.` `..` `.-------.```.-. `.---------------------------------------------------`      
//                  `/sssso/-----------.  shhs` .-` -yhhh/....:::hhy//+yyy. /so/  .::-` `--`     ``.--......  -/++o. `  /hy` `...``..`` .shh-` `.---------------------------------------------------`     
//                 .+ssss+-------------` -hhh/ `--.` .+syyyso+:``-//:.`..`` `        ``````` .-. `.--------.  /hhhhs`   oh/   `.---.` ./ohhyoo` `---------------------------------------------------.     
//               `:osss+:-------------.  osso- `----.````..`````.````.......  -++o+   `.-`  -yhs` ```````..` `yhshhho` -hy``-+so/+hhs..:hhh-`` `.----------------------------------------------------     
//             `:ossso:---------------.` ``````.------........-------------.` :hhh:  .syy.` .+o. `-:/++`  `  :hs.yhhh+`oh/`+hh/` `yhh/ /hho  ..------------------------------------------------------.    
//            .+ssss+-------------------....----------------------------...`  ohhy`./shhyo+`--`.oyo::-:  `  `sh: -hhhh+hy.ohho`  .hhh-`yhh.  `...-----------------------------------------------------`   
//          .+sssso/-----------------------------------------------...`````  `hhh: -/hhy-``   -yhh/-.`   `  -hs`  /hhhhh+-hhh-  `ohy/ /hhy.`  `````....-----------------------------------------------.   
//        `:sssss+:--------------------------------------------..``````````  .hhy`  +hh/  ``  .oyyhhhs-  `  sh:   `shhhy.-yhh:.:sy+.  /yyys/  ``````````..---------------------------------------------`  
//       `+sssss+------------------------------------------..`````````````   ohh/  .hhy`  `     `.:yhy-    -hy`    .yyy+  .+ooo+:`     ```   ``````````````.-------------------------------------------`  
//      .osssss+---------------------------------------...```````````````   -hhy`  +hho``   .+/::/oyo-     /+/  ``  ...`      ``````       ``````````````````------------------------------------------`  
//     .osssss+-------------------------------------..```````````````````   ohh+   ohhys:   .////:-`   `````````````````````     ```   ``   ``````````````````.----------------------------------------`  
//    `osssss+-----------------------------------..``````````````````````   :/:-    ```                    ````     ```````  -:`  ``  .yho  ```````````````````.---------------------------------------`  
//    /ssssso---------------------------------.```````````````````````````       ````````````     .-://+++osyhh+    ``````` :hhs   `  -hhy` ````````````````````.--------------------------------------`  
//   :ssssss:-------------------------------.````````````````````````````````````````````````    +hhhhhhhhhys+/.    ``````` shh/   `` .hhh.  ````````````````````.------------------------------------.   
//  -ssssss/------------------------------.`````````````````````````````````````````````````    /hhy-..``         ```````` .hhh.  ```  yhh:  `````````````````````.-----------------------------------`   
// .ossssso-----------------------------.```````````````````````    `-/////:.    ```````````    shh/   ``````````````````  +hho   ```  +hho  `````````````````````.----------------------------------.    
//`+ssssss/---------------------------.`````````````````````   `-/oyhhhhhhhhhhs/.   ````````    hhh-````````````````````  .hhh.  ```   :hhy    ````````````````````----------------------------------`    
//-sssssss---------------------------.````````````````````  .+yhhhhyo/-..--:+shhh/   ```````   -hhy````````````````````  `yhh+     ``.-/hhh+-   ```````````````````---------------------------------.     
//ossssss+-------------------------.`````````````````````  -hhho/-`           .yhhs`  `````    +hho            ```````   +hhy::+osyhhhhhhhhho  ````````````````````--------------------------------.      
//sssssss/------------------------```````````````````````` `::`   ```````````` `ohhs`  ````    shh+-/osyhys+:.    ```   :hhhhhhhhys+//:/hhh`  `````````````````````-------------------------------.`      
//sssssss:----------------------.``````````````````````````    ```````````````` `yhh/  ````   `hhhhhhhyooshhhhs:    `   +hhs+:-.```    .hhh` `````````````````````.-------------------------------`       
//sssssss:----------------------```````````````````````````````````````````````  +hho  ````   `sys+:-``` ``./yhho`       ``   ```````  .hhh. ````````````````````.-------------------------------`        
//sssssss:---------------------.```````````````````````````````````````````````  ohho  ````     `  ```````````ohhs`  ````````````````  `yhh- ````````````````````------------------------------.`         
//sssssss/---------------------.``````````````````````````````````````````````` -hhh-  `````      `````````````ohho   ````````````````  shh/ ```````````````````.-----------------------------.`          
//sssssss+---------------------``````````````````````````````````````````````` `yhh/   ``````````````````````` .hhh.  ````````````````  shh/ ``````````````````.-----------------------------.            
//ssssssss---------------------`````````````````````````````````````````````` `shho`  ```````````````````````` `hhh.  ````````````````  shh/ ````````````````.-----------------------------//             
//ssssssss/--------------------.```````````````````````````````````````````  `shho`  ````````````````````````` -hhh`  ````````````````  shh/ ```````````````.---------------------------:+oso.            
//sssssssso---------------------`````````````````````````````````````````  `:yhho`  ``````````     ``````````  ohho   ````````````````  +hh+ `````````````.---------------------------/osssss-            
//sssssssss+---------------------`````````````````````````````````````   ./yhhs/` ```````````       ````````` /hhy.  `````````````````  /hhs ``````````..--------------------------:+osssssss-            
//osssssssss/---------------------.``````````````````````````````      -+yhhs:`  ````     ``    .:-         .ohhy-   `````````````````  -yh+  ```````..-------------------------:+ossssssssss:            
//:ssssssssss+---------------------..````````````````````````       `:ohhyo-`                   shho/-...-:oyhho`   ``````````````````   ..` `````..-------------------------:+osssssssssssss:            
//`/ssssssssss+:---------------------..```````````````````      `.:oyhhho:......--:::/++ooo:    .+syhhhhhhhhs+-   `````````````````````     ```..------------------------:/+ossssssssssssssss:            
// `osssssssssso/-----------------------.````````````````    ./syhhhhhhhyyyhhhhhhhhhhhyyyys/   `  `.-////::.`   ````````````````````````````..-----------------------::+ossssssssssssssssssss-            
//  -osssssssssss+:-----------------------..`````````````    +hhysssooo+++//////::--...````   `````````````````````````````````````````...-----------------------::+oosssssssssssssssssssssso`            
//   .ossssssssssss+:-----------------------..````````````   `...``                        ````````````````````````````````````````...-----------------------:/+oossssssssssssssssssssssssss-             
//    .ossssssssssssso/:-----------------------....``````````       ````````````````````````````````````````````````````````....------------------------:/+oossssssssssssssssssssssssssssso.              
//     .osssssssssssssso+/---------------------------...````````````````````````````````````````````````````````````......------------------------::/+oosssssssssssssssssssssssssssssssso-`               
//      `+ssssssssssssssssoo/:-------------------------------.....``````````````````````.......................-----------------------------::/++oossssssssssssssssssssssssssssssssssss:`                 
//       `/sssssssssssssssssss+/:--------------------------------------....----------------------------------------------------------:://+oosssssssssssssssssssssssssssssssssssssssss+.                   
//         .ossssssssssssssssssssoo+/::----------------------------------------------------------------------------------------://+oossssssssssssssssssssssssssssssssssssssssssssso/.                     
//          `:ossssssssssssssssssssssssoo++//::-------------------------------------------------------------------------::/+ooosssssssssssssssssssssssssssssssssssssssssssssssso/-`                       
//            `-+ssssssssssssssssssssssssssssssooo++//::-----------------------------------------------::::::::///++ooosssssssssssssssssssssssssssssssssssssssssssssssssssss+:.`                          
//              `-/osssssssssssssssssssssssssssssssssssssooo+//////::::::::::///////+++++++++++oooooossssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssso/-.                              
//                 `:+ossssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssso+:.`                                 
//                    .:+osssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssoo+:-.``                                     
//                       `-/ossssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssso+:-.`                                            
//                           `-/+osssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssoo+:.`                                                 
//                               `.-:/osssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssso/:-.`                                                       
//                                     `.-:/ossssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssso+/:-.`                                                             
//                                           `..-:/+oossssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssoo+//:-..`                                                                    
//                                                    ``..-:://++oooosssssssssssssssssssssssssssssssssssssssoooo+++/:-..````                                                                              
//                                                                   `````.........------::---------.....````                                                                                             