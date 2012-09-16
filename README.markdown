Introduction
------------

**AMa_ChamferMaker** is a generator plugin that allows you to add  
beveled edges to any 3D object, procedural or polygonal, while  
keeping the object manager tree fully procedural. This means, you  
can get nice accentuated edges that will make your renders look  
much more realistic while keeping the power to change things after  
the fact.

It also has a few extra tricks under its sleeve for dealing with  
geometry a bit more complicated.

Installation
------------

Compile and install to CINEMA 4D's plugin folder  
(or the equivalent in the user preferences folder).

If you do not want to compile the plugin, you can find  
a pre-compiled binary distribution in the downloads  
section. 

Supported Versions
------------------

The current version, named `AMa_Chamfer_Maker-R14.zip` is for R14,  
but you can also find the previous version for R12/R13 in the downloads  
section named `AMa_Chamfer_Maker.zip`.

Development
-----------

If you want to use the included Xcode 4/VisualStudio 2012 projects  
it is crucial to put the downloaded source folder inside CINEMA 4D's  
plugin directory, under the main installation path (as opposed to   
the plugin folder found in user prefs). This is because all paths  
relevant to the compiler/linker are set up relative to the main  
plugin folder.


Basic Usage
-----------

Go to the *Plugins* menu, choose *AMa_ChamferMaker*.  
This will instantiate the generator object in the Object Manager.  

From hereon you can put the model objects you want bevelled under the  
generator object just as you would with any generator (e.g. a HyperNURBS).

In the generators Attribute Manager interface set the targeting mode  
and bevel away.


Attribution
-----------

Created by Antosha Marchenko in 2006.  
Updated from the 2006 source code by yours truly.

Copyright
---------

Copyright 2006, Antosha Marchenko  
Copyright 2011, 2012, André Berg

License
-------

Since I couldn't find a license notice with the source I downloaded,  
I am putting it under the following license:

Licensed under the Apache License, Version 2.0 (the "License");  
you may not use this file except in compliance with the License.  
You may obtain a copy of the License at

[http://www.apache.org/licenses/LICENSE-2.0](http://www.apache.org/licenses/LICENSE-2.0)

Unless required by applicable law or agreed to in writing, software  
distributed under the License is distributed on an "AS IS" BASIS,  
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  
See the License for the specific language governing permissions and  
limitations under the License.  
