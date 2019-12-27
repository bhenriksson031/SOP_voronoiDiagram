

#include "SOP_VoronoiGraph.h"

// This is an automatically generated header file based on theDsFile, below,
// to provide SOP_VoronoiGraphParms, an easy way to access parameter values from
// SOP_VoronoiGraphVerb::cook with the correct type.
#include "SOP_VoronoiGraph.proto.h"

#include <GU/GU_Detail.h>
#include <GEO/GEO_PrimPoly.h>
#include <OP/OP_Operator.h>
#include <OP/OP_OperatorTable.h>
#include <PRM/PRM_Include.h>
#include <PRM/PRM_TemplateBuilder.h>
#include <UT/UT_DSOVersion.h>
#include <UT/UT_Interrupt.h>
#include <UT/UT_StringHolder.h>
#include <SYS/SYS_Math.h>
#include <limits.h>

#include <OP/OP_AutoLockInputs.h>
#include <include/houdiniVoronoiGraph.h>


using namespace HDK_Beh;


/// This is the internal name of the SOP type.
/// It isn't allowed to be the same as any other SOP's type name.
const UT_StringHolder SOP_VoronoiGraph::theSOPTypeName("beh_voronoiGraph2d"_sh);

/// newSopOperator is the hook that Houdini grabs from this dll
/// and invokes to register the SOP.  In this case, we add ourselves
/// to the specified operator table.
void
newSopOperator(OP_OperatorTable *table)
{
    table->addOperator(new OP_Operator(
        "voronoiGraph",   // Internal name
        "Voronoi Graph",                     // UI name
        SOP_VoronoiGraph::myConstructor,    // How to build the SOP
        SOP_VoronoiGraph::myTemplateList, // My parameters
        1,                          // Min # of sources
        1,                          // Max # of sources
        nullptr,                    // Custom local variables (none)
        NULL));        // Flag it as generator
}

/// This is a multi-line raw string specifying the parameter interface
/// for this SOP.
static const char *theDsFile = R"THEDSFILE(
{
    name        parameters
    parm {
        name    "divs"      // Internal parameter name
        label   "Divisions" // Descriptive parameter name for user interface
        type    integer
        default { "5" }     // Default for this parameter on new nodes
        range   { 2! 50 }   // The value is prevented from going below 2 at all.
                            // The UI slider goes up to 50, but the value can go higher.
        export  all         // This makes the parameter show up in the toolbox
                            // above the viewport when it's in the node's state.
    }
    }
}
)THEDSFILE";

static PRM_Name names[] = {
	PRM_Name("divs",	"Divisions"),
	PRM_Name("bound",	"Boundary Distance"),
	PRM_Name("mag",	"Mag"),
	PRM_Name("doBoundaryGeo",	"Boundary Geo"),
	PRM_Name("resampleCurves",	"resampleCurves"),
	PRM_Name("keepOutsideSegments",	"keepOutsideSegments"),
	PRM_Name("removePointSegments",	"removePointSegments"),


};
PRM_Template
SOP_VoronoiGraph::myTemplateList[] = {
	PRM_Template(PRM_INT,  1, &names[0], PRMzeroDefaults),
	PRM_Template(PRM_FLT_J,	1, &names[1], PRMzeroDefaults, 0, &PRMscaleRange),
	PRM_Template(PRM_FLT_J,	1, &names[2], PRMzeroDefaults, 0, &PRMscaleRange),
	PRM_Template(PRM_TOGGLE,    1, &names[3]),
	PRM_Template(PRM_TOGGLE,    1, &names[4]),
	PRM_Template(PRM_TOGGLE,    1, &names[5]),
	PRM_Template(PRM_TOGGLE,    1, &names[6]),
	PRM_Template(PRM_DIRECTION, 3, &PRMdirectionName, PRMzaxisDefaults),
	PRM_Template(),
};

OP_ERROR
SOP_VoronoiGraph::cookMySop(OP_Context &context)
{
	//printf("SOP_VoronoiGraphVerb::cook...\n");
	OP_AutoLockInputs inputs(this);
	if (inputs.lock(context) >= UT_ERROR_ABORT)
		return error();

	fpreal now = context.getTime();

	duplicateSource(0, context);
	// 2. Copy input geometry into our gdp
	// 3. Parse and create myGroup
	if (cookInputGroups(context) >= UT_ERROR_ABORT)
		return error();

	setCurGdh(0, myGdpHandle);

	boostVoronoiGraph vor_diagram;
	double bound = GETBOUND();
	double mag = GETMAG();
	double do_boundary_geo = GETDOBOUNDARYGEO();
	bool do_resample = GETRESAMPLE();
	bool keep_outside_segments = GETKEEPOUTIDES();
	bool remove_point_segments = GETREMOVEPOINTSEGS();
	vor_diagram.addVoronoiGraphToHoudiniGeo(gdp, bound, mag, do_boundary_geo, do_resample, keep_outside_segments, remove_point_segments);

	inputs.unlock();
	return error();

}

