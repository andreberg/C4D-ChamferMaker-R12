/////////////////////////////////////////////////////////////
// CINEMA 4D SDK                                           //
/////////////////////////////////////////////////////////////
// (c) 1989-2004 MAXON Computer GmbH, all rights reserved  //
/////////////////////////////////////////////////////////////

// Starts the plugin registration

#include "c4d.h"
#include <string.h>

// forward declarations
/*Bool RegisterSN();
Bool RegisterGradient(void);
Bool RegisterBitmap(void);
Bool RegisterMandelbrot(void);
Bool RegisterSimpleMaterial(void);
Bool RegisterParticleVolume(void);
Bool RegisterMenuTest(void);
Bool RegisterAsyncTest(void);
Bool RegisterActiveObjectDlg();
Bool RegisterListView(void);
Bool RegisterSubDialog(void);

Bool RegisterRoundedTube(void);

Bool RegisterVPTest(void);
Bool RegisterVPInvertImage(void);
Bool RegisterBlinker(void);

Bool RegisterCircle(void);
Bool RegisterSTL(void);
Bool RegisterBFF(void);
Bool RegisterLookAtCamera(void);
Bool RegisterGravitation(void);
Bool RegisterThreshold(void);
Bool RegisterSampleMatrix(void);
Bool RegisterPrimitiveTool(void);
Bool RegisterMorphMixer(void);
/*Bool RegisterVPVisualizeNormals(void);
Bool RegisterVPReconstruct(void);
Bool RegisterExampleDataType(void);
void FreeTriangulate(void);
//Bool RegisterMemoryStat(void);
Bool RegisterEdgeCutTool();*/
//Bool RegisterReverseNormals();
//Bool RegisterAtomObject(void);
Bool Register_AMa_Deformer(void);
//Bool RegisterTriangulate(void);
/*Bool RegisterLayerShaderBrowser();
Bool RegisterPainterSaveTest();
Bool RegisterRandomFalloff();
Bool RegisterNoiseEffector();
Bool RegisterDropEffector();*/

C4D_CrashHandler old_handler;

void SDKCrashHandler(CHAR *crashinfo)
{
	//printf("SDK CrashInfo:\n");
	//printf(crashinfo);
	
	// don't forget to call the original handler!!!
	if (old_handler) (*old_handler)(crashinfo);
}

void EnhanceMainMenu(void) 
{
	// do this only if necessary - otherwise the user will end up with dozens of menus!

	if ((bool)GeGetVersionType() < 0) // only if C4D is loaded
		return;

	BaseContainer *bc = GetMenuResource(String("M_EDITOR"));
	if (!bc) return;

	// search for the most important menu entry. if present, the user has customized the settings
	// -> don't add menu again
	if (SearchMenuResource(bc,String("PLUGIN_CMD_1000472")))
		return; 

	GeData *last = SearchPluginMenuResource();

	BaseContainer sc;
	sc.InsData(MENURESOURCE_SUBTITLE,String("SDK Test"));
	sc.InsData(MENURESOURCE_COMMAND,String("IDM_NEU")); // add C4D's new scene command to menu
	sc.InsData(MENURESOURCE_SEPERATOR,TRUE);
	sc.InsData(MENURESOURCE_COMMAND,String("PLUGIN_CMD_1000472")); // add ActiveObject dialog to menu
	
	if (last)
		bc->InsDataAfter(MENURESOURCE_STRING,sc,last);
	else // user killed plugin menu - add as last overall entry
		bc->InsData(MENURESOURCE_STRING,sc);
}

Bool PluginStart(void)
{
	// example of installing a crashhandler
	old_handler = C4DOS.CrashHandler; // backup the original handler (must be called!)
	C4DOS.CrashHandler = SDKCrashHandler; // insert the own handler
	
	C4DPL_CommandLineArgs args;
	if (GetCommandLineArgs(args))
	{
		GeConsoleOut("COMMANDLINE number of arguments: "+LongToString(args.argc)); 
	}

	// shader plugins
	/*if (!RegisterGradient()) return FALSE;
	if (!RegisterBitmap()) return FALSE;
	if (!RegisterMandelbrot()) return FALSE;
	if (!RegisterSimpleMaterial()) return FALSE;
	if (!RegisterParticleVolume()) return FALSE;

	// menu plugins
	if (!RegisterMenuTest()) return FALSE;
	if (!RegisterAsyncTest()) return FALSE;
	if (!RegisterActiveObjectDlg()) return FALSE;
	if (!RegisterListView()) return FALSE;
	if (!RegisterSubDialog()) return FALSE;
	if (!RegisterLayerShaderBrowser()) return FALSE;

	// filter plugins
	if (!RegisterSTL()) return FALSE;
	if (!RegisterBFF()) return FALSE;

	// object plugins
	
	if (!RegisterRoundedTube()) return FALSE;
	//if (!RegisterGravitation()) return FALSE;
	
	if (!RegisterCircle()) return FALSE;
	
	if (!RegisterMorphMixer()) return FALSE;

	// tool plugins
	if (!RegisterPrimitiveTool()) return FALSE;
	if (!RegisterEdgeCutTool()) return FALSE;*/
	//if (!RegisterReverseNormals()) return FALSE;
	//if (!RegisterTriangulate()) return FALSE;
	//if (!RegisterAtomObject()) return FALSE;
	if (!Register_AMa_Deformer()) return FALSE;

	// animation plugins
	/*if (!RegisterBlinker()) return FALSE;

	// tag / expression plugins
	if (!RegisterLookAtCamera()) return FALSE;

	// bitmap filter
	if (!RegisterThreshold()) return FALSE;
	if (!RegisterSampleMatrix()) return FALSE;

	// video post filter
	if (!RegisterVPTest()) return FALSE;
	if (!RegisterVPInvertImage()) return FALSE;
	if (!RegisterVPVisualizeNormals()) return FALSE;
	if (!RegisterVPReconstruct()) return FALSE;

	if (!RegisterMemoryStat()) return FALSE;
	if (!RegisterPainterSaveTest()) return FALSE;

	// falloff types
	if (!RegisterRandomFalloff()) return FALSE;

	// effector plugins, can only be loaded if MoGfx is installed
	RegisterNoiseEffector();
	RegisterDropEffector();*/

	return TRUE;
}

void PluginEnd(void)
{
}

Bool PluginMessage(LONG id, void *data)
{
	//use the following lines to set a plugin priority
	//
	switch (id)
	{
		case C4DPL_INIT_SYS:
			if (!resource.Init()) return FALSE; // don't start plugin without resource
			//if (!RegisterExampleDataType()) return FALSE;
			return TRUE;

		case C4DMSG_PRIORITY: 
			return TRUE;

		case C4DPL_BUILDMENU:
			//EnhanceMainMenu(); 	
			break;
			
		case C4DPL_COMMANDLINEARGS:
			{
				C4DPL_CommandLineArgs *args = (C4DPL_CommandLineArgs*)data;
				LONG i;

				for (i=0;i<args->argc;i++)
				{
					if (!args->argv[i]) continue;
					
					if (!strcmp(args->argv[i],"--help") || !strcmp(args->argv[i],"-help"))
					{
						// do not clear the entry so that other plugins can make their output!!!
						GePrint("\x01-SDK is here :-)");
					}
					else if (!strcmp(args->argv[i],"-SDK"))
					{
						args->argv[i] = NULL;
						GePrint("\x01-SDK executed:-)");
					}
					else if (!strcmp(args->argv[i],"-plugincrash"))
					{
						args->argv[i] = NULL;
						*((LONG*)0) = 1234;
					}
				}
			}
			break;

		case C4DPL_EDITIMAGE:
			{
				C4DPL_EditImage *editimage = (C4DPL_EditImage*)data;
				if (!data) break;
				if (editimage->return_processed) break;
				GePrint("C4DSDK - Edit Image Hook: "+editimage->imagefn->GetString());
				// editimage->return_processed = TRUE; if image was processed
			}
			return FALSE;
	}

	return FALSE;
}
