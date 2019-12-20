/*
 * Copyright (c) 2019
 *	Side Effects Software Inc.  All rights reserved.
 *
 * Redistribution and use of Houdini Development Kit samples in source and
 * binary forms, with or without modification, are permitted provided that the
 * following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. The name of Side Effects Software may not be used to endorse or
 *    promote products derived from this software without specific prior
 *    written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY SIDE EFFECTS SOFTWARE `AS IS' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 * NO EVENT SHALL SIDE EFFECTS SOFTWARE BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *----------------------------------------------------------------------------
 * This SOP builds a voronoi graph.
 */

#ifndef __SOP_VoronoiGraph_h__
#define __SOP_VoronoiGraph_h__

#include <SOP/SOP_Node.h>
#include <UT/UT_StringHolder.h>

namespace HDK_Beh {
/// This is the SOP class definition.  It doesn't need to be in a separate
/// file like this.  This is just an example of a header file, in case
/// another file needs to reference something in here.
/// You shouldn't have to change anything in here except the name of the class.
class SOP_VoronoiGraph : public SOP_Node
{
public:
	static PRM_Template		 myTemplateList[];
    static PRM_Template *buildTemplates();
    static OP_Node *myConstructor(OP_Network *net, const char *name, OP_Operator *op)
    {
        return new SOP_VoronoiGraph(net, name, op);
    }

    static const UT_StringHolder theSOPTypeName;
    
    //virtual const SOP_NodeVerb *cookVerb() const override;

protected:
    SOP_VoronoiGraph(OP_Network *net, const char *name, OP_Operator *op)
        : SOP_Node(net, name, op)
    {
        // All verb SOPs must manage data IDs, to track what's changed
        // from cook to cook.
        mySopFlags.setManagesDataIDs(false); //manual edit, was set to false!
    }
	/// Method to cook geometry for the SOP
	virtual OP_ERROR		 cookMySop(OP_Context &context);

    virtual ~SOP_VoronoiGraph() {}
	/* not a verb anymore
    /// Since this SOP implements a verb, cookMySop just delegates to the verb.
    virtual OP_ERROR cookMySop(OP_Context &context) override
    {
        return cookMyselfAsVerb(context);
    }
	*/
private: 
	double GETBOUND() { return evalFloat("bound", 0, 0); }
	double GETMAG() { return evalFloat("mag", 0, 0); }
	bool GETDOBOUNDARYGEO() { return evalInt("doBoundaryGeo", 0, 0) == 1 ? true : false;}
	bool GETRESAMPLE() { return evalInt("resampleCurves", 0, 0) == 1 ? true : false;}
	bool GETREMOVEOUTIDES() { return evalInt("removeOutsideSegments", 0, 0) ==1 ? true:false; }
};
} // End HDK_Sample namespace

#endif
