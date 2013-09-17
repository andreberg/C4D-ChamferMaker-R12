#include "c4d.h"
#include "c4d_memory.h"
#include "c4d_symbols.h"
#include "lib_modeling.h"
#include "lib_ngon.h"
#include "customgui_inexclude.h"
#include "O_AMa_ChamferMaker.h"

#if API_VERSION < 15000
	#include "toolbevel.h"
	#define _ID_BEVELTOOL ID_MODELING_BEVEL_TOOL
#else
	#include "xbeveltool.h"
	#define _ID_BEVELTOOL 431000015
#endif

const LONG AMaPOINT_MAP_MAX_BRICKS = 50;

struct AMaPointMap_struct {
    LONG * map_plus, * map_minus[AMaPOINT_MAP_MAX_BRICKS], bricksAllocated, cntInBr;

    AMaPointMap_struct() {
        map_plus = NULL;
        for ( LONG i = 0; i < AMaPOINT_MAP_MAX_BRICKS; i++) {
            map_minus[i] = NULL;
        }
    };

    Bool AllocAMaPointMap(LONG * map, LONG cnt, LONG objPntCnt) {
        LONG y;

        map_minus[0] = (LONG *)GeAlloc(sizeof(LONG) * objPntCnt);
        if ( !map_minus[0]) {
            return false;
        }
        for ( y = 0; y < objPntCnt; y++) {
            map_minus[0][y] = -1;
        }

        map_plus = (LONG *)GeAlloc(sizeof(LONG) * objPntCnt);
        if ( !map_plus) {
            return false;
        }
        for ( y = 0; y < objPntCnt; y++) {
            map_plus[y] = -1;
        }

        bricksAllocated = 1;
        cntInBr = objPntCnt;
        for ( LONG i = 0; i < cnt; i++) {
            if ( map[ i * 2] > objPntCnt) {
                return false;
            }
            if ( map[ i * 2] > 0) {
                map_plus[ map[ i * 2]] = map[ i * 2 + 1];
            } else {
                LONG brick = -map[ i * 2] / objPntCnt;
                if ( brick < bricksAllocated) {
                    map_minus[brick][ -map[ i * 2] - brick * objPntCnt] = map[ i * 2 + 1];
                } else {
                    if ( brick > AMaPOINT_MAP_MAX_BRICKS) {
                        return false;
                    }
                    for ( LONG j = bricksAllocated; j <= brick; j++) {
                        map_minus[j] = (LONG *)GeAlloc(sizeof(LONG) * objPntCnt);
                        if ( !map_minus[j]) {
                            return false;
                        }
                        for ( y = 0; y < objPntCnt; y++) {
                            map_minus[j][y] = -1;
                        }
                    }
                    bricksAllocated = brick + 1;
                    map_minus[brick][ -map[ i * 2] - brick * objPntCnt] = map[ i * 2 + 1];
                }
            }
        }
        return true;
    };

    void Free() {
        GeFree(map_plus);
        for ( LONG i = 0; i < bricksAllocated; i++) {
            GeFree(map_minus[i]);
        }
    };

    LONG GetNum(LONG p) {
        LONG result;

        if ( p > 0) {
            result = map_plus[p];
        } else {
            LONG brick = -p / cntInBr;
            result = map_minus[brick][ -p - brick * cntInBr];
        }
        if ( result == -1) {
            return p;
        } else {
            return result;
        }
    };

    void PrintMinus1() {
        for ( LONG i = 0; i < cntInBr; i++) {
            GePrint(LongToString(i) + "   " + LongToString(map_minus[0][i]));
        }
    };
};

Bool chk;

Bool IsEdgeDoubled(Neighbor * nbr, LONG e);
void Get_x4_edge_points(const CPolygon * Plgons, LONG E_In_P, LONG P_of_E, LONG &pA, LONG &pB);
void Get_x4_edge_points(const CPolygon * Plgons, LONG e, LONG &pA, LONG &pB);
Bool Get_H_topol_points(Modeling * krnl, PolygonObject * obj, LONG pA, LONG pB, Bool &valid, LONG &wingAL, LONG &wingAR, Bool &opened, LONG &wingBL, LONG &wingBR);
LONG PredPointIndInNgn(Ngon * Ngn, LONG p);
LONG NextPointIndInNgn(Ngon * Ngn, LONG p);
Bool findPredPointInNgn(Ngon * Ngn, LONG p, LONG &prP);
Bool findNextPointInNgn(Ngon * Ngn, LONG p, LONG &prP);
Bool findNextPointAlongNgn(Modeling * krnl, PolygonObject * obj, LONG p1, LONG p2, LONG p3, LONG &p4);
// Bool Compare_pBaseObjects( BaseObject *Ob, BaseObject *cacheOb) ;
void SelectOpenEdges(PolygonObject * obj, Neighbor * nbr, BaseSelect * selE);
void SelectEdgesWithThrAngle(PolygonObject * obj, Neighbor * nbr, Modeling * krnl, BaseSelect * selE, Real ang, Bool andOpen);
LONG Point_From_Plg(const CPolygon * Plg, LONG i);
LONG GetEdgeFromPoints(const CPolygon * plgns, Neighbor * nbr, LONG p1, LONG p2);
Bool GetNgnFrom3pnts(Modeling * krnl, PolygonObject * obj, LONG p1, LONG p2, LONG p3, LONG &ngn);
Bool GetNextPointInFan(Modeling * krnl, PolygonObject * obj, LONG pc, LONG p1, LONG p2, LONG &p3, Bool &finded);
LONG SplitEdge_AbsDist(Modeling * krnl, PolygonObject * obj, LONG p1, LONG p2, Real ChamfRadius);
LONG SplitEdge_AbsDist(Modeling * krnl, PolygonObject * obj, LONG p1, LONG p2, Real ChamfRadius, Bool &tooClose);
Bool ConnectPnts(Modeling * krnl, PolygonObject * obj, LONG p1, LONG p2pl1, LONG pl2, LONG pl3);


class AMaChamMaker : public ObjectData {
public:
virtual Bool Init(GeListNode * node);
virtual Bool GetDDescription(GeListNode * node, Description * description, DESCFLAGS_DESC& flags);
virtual BaseObject * GetVirtualObjects(BaseObject * op, HierarchyHelp * hh);

static NodeData * Alloc(void) {
    return gNew AMaChamMaker;
}
private:
PolygonObject * obj;
BaseContainer * data;
BaseSelect * selE;
Modeling * krnl;
Neighbor nbr;
Bool NbrInited;
Bool krnlInited;

Bool Recurse_All_Childs(BaseObject * gener, BaseThread * bt, BaseObject * op, String s_hierarPath);
Bool Recurse_First_Child(BaseObject * gener, BaseThread * bt, BaseObject * op, BaseObject * main, String s_hierarPath);
Bool ModifyObj(BaseObject * mod, BaseObject * op, BaseObject * realObj);
Bool MakeChamf_In_AMa_PARALLEL_Mode(Bool CompositeModeOn, Bool DoubleParallel, Bool CompositeFirstPass);
HNWeightTag * get_HN_tag();
};

// ----------Init-----------------------------------------------------------------------------------
Bool AMaChamMaker::Init(GeListNode * node) {
    BaseObject * op = (BaseObject *)node;

    data = op->GetDataInstance();

    data->SetBool(AMa_CHMMKR_ALL_CHILDREN, FALSE);
    data->SetLong(AMa_CHMMKR_SEL_MODE, AMa_CHMMKR_SELMO_LIVE);
    data->SetLong(AMa_CHMMKR_N_TH_SS, 1);
    data->SetReal(AMa_CHMMKR_SELECT_THRESH, 60);
    data->SetBool(AMa_CHMMKR_AND_OPEN_EDG, FALSE);
    data->SetReal(AMa_CHMMKR_RAD_a, 5.0);
    data->SetReal(AMa_CHMMKR_RAD_p, 5.0);
    data->SetReal(AMa_CHMMKR_RAD_c, 5.0);
    data->SetReal(AMa_CHMMKR_VARIANCE, 0.0);
    data->SetLong(AMa_CHMMKR_SUBDIVISION, 0);
    data->SetBool(AMa_CHMMKR_CREATENGONS, TRUE);
    data->SetLong(AMa_CHMMKR_SHAPE_MODE, AMa_CHMMKR_MODE_LINEAR);
    AutoAlloc<SplineData> falloffspline;
    if ( !falloffspline) {
        return FALSE;
    }
    falloffspline->InsertKnot(0.0, 0.0);
    falloffspline->InsertKnot(0.441, 0.742);
    falloffspline->InsertKnot(1.0, 1.0);
    // falloffspline->SetRound(0.34);
    data->SetData(AMa_CHMMKR_PATH, GeData(CUSTOMDATATYPE_SPLINE, falloffspline));
    data->SetReal(AMa_CHMMKR_EXTRUSION, 6.0);
    data->SetReal(AMa_CHMMKR_EXTRU_VARI, 0.0);
    data->SetBool(AMa_CHMMKR_CLOSE_FANS, FALSE);
    data->SetBool(AMa_CHMMKR_PROL_OUTL, FALSE);
    data->SetBool(AMa_CHMMKR_CONNECT_OUTL, FALSE);
    data->SetReal(AMa_CHMMKR_CONN_THRESH, 4.0);
    //
    data->SetReal(AMa_CHMMKR_RAD_PERC, 1.5);
    data->SetBool(AMa_CHMMKR_COMP_CLO_F, FALSE);
    data->SetBool(AMa_CHMMKR_COMP_PR_O, FALSE);
    data->SetBool(AMa_CHMMKR_COMP_CONN, FALSE);
    data->SetReal(AMa_CHMMKR_COMP_CO_THR, 4.0);
    data->SetLong(AMa_CHMMKR_COMP_SEC_MO, AMa_CHMMKR_MODE_LINEAR);
    //
    data->SetLong(AMa_CHMMKR_HN_WHAT, AMa_CHMMKR_HN_OFF);
    data->SetLong(AMa_CHMMKR_HN_PRIORITY, AMa_CHMMKR_HN_BIG);
    data->SetBool(AMa_CHMMKR_HN_RESET, FALSE);
    return TRUE;
}

// ----------GetDDescription----------------------------------------------------------------------------
Bool AMaChamMaker::GetDDescription(GeListNode * node, Description * description, DESCFLAGS_DESC &flags) {
    if ( !description || !node) {
        return FALSE;
    }
    if ( !description->LoadDescription(node->GetType())) {
        return FALSE;
    }
    AutoAlloc<AtomArray> ar;
    if ( !ar) {
        return FALSE;
    }

    BaseObject * op = static_cast<BaseObject *>( node);
    BaseContainer * bc;
    data = op->GetDataInstance();
    ar->Append(static_cast<C4DAtom *>(node));

    if ( data->GetLong(AMa_CHMMKR_SEL_MODE) != AMa_CHMMKR_SELMO_CHOOSE) {
        bc = description->GetParameterI(DescLevel(AMa_CHMMKR_IN_EXCL), ar);
        if ( bc) {
            bc->SetBool(DESC_HIDE, TRUE);
        }
    }
    if ( data->GetLong(AMa_CHMMKR_SEL_MODE) != AMa_CHMMKR_SELMO_NSSET) {
        bc = description->GetParameterI(DescLevel(AMa_CHMMKR_N_TH_SS), ar);
        if ( bc) {
            bc->SetBool(DESC_HIDE, TRUE);
        }
    }
    if ( data->GetLong(AMa_CHMMKR_SEL_MODE) != AMa_CHMMKR_SELMO_THRESH) {
        bc = description->GetParameterI(DescLevel(AMa_CHMMKR_SELECT_THRESH), ar);
        if ( bc) {
            bc->SetBool(DESC_HIDE, TRUE);
        }

        if ( data->GetLong(AMa_CHMMKR_SEL_MODE) != AMa_CHMMKR_SELMO_PHO) {
            bc = description->GetParameterI(DescLevel(AMa_CHMMKR_AND_OPEN_EDG), ar);
            if ( bc) {
                bc->SetBool(DESC_HIDE, TRUE);
            }
        }
    }
    // /////////////////////////////
    switch ( data->GetLong(AMa_CHMMKR_SHAPE_MODE)) {
        case AMa_CHMMKR_MODE_AMaPARAL:
            bc = description->GetParameterI(DescLevel(AMa_CHMMKR_RAD_PERC), ar);
            if ( bc) {
                bc->SetBool(DESC_HIDE, TRUE);
            }
            bc = description->GetParameterI(DescLevel(AMa_CHMMKR_GROUP_COMP), ar);
            if ( bc) {
                bc->SetBool(DESC_HIDE, TRUE);
            }
            bc = description->GetParameterI(DescLevel(AMa_CHMMKR_GROUP_STANDARD), ar);
            if ( bc) {
                bc->SetBool(DESC_HIDE, TRUE);
            }
            if ( !data->GetBool(AMa_CHMMKR_CONNECT_OUTL)) {
                bc = description->GetParameterI(DescLevel(AMa_CHMMKR_CONN_THRESH), ar);
                if ( bc) {
                    bc->SetBool(DESC_HIDE, TRUE);
                }
            }
            if (( data->GetLong(AMa_CHMMKR_HN_WHAT) == AMa_CHMMKR_HN_OFF) ||
                ( data->GetLong(AMa_CHMMKR_HN_WHAT) == AMa_CHMMKR_HN_P_ONLY) ) {
                bc = description->GetParameterI(DescLevel(AMa_CHMMKR_HN_PRIORITY), ar);
                if ( bc) {
                    bc->SetBool(DESC_HIDE, TRUE);
                }
            }
            break;
        case AMa_CHMMKR_MODE_COMPO:
            bc = description->GetParameterI(DescLevel(AMa_CHMMKR_RAD_p), ar);
            if ( bc) {
                bc->SetBool(DESC_HIDE, TRUE);
            }
            if ( !data->GetBool(AMa_CHMMKR_CONNECT_OUTL)) {
                bc = description->GetParameterI(DescLevel(AMa_CHMMKR_CONN_THRESH), ar);
                if ( bc) {
                    bc->SetBool(DESC_HIDE, TRUE);
                }
            }
            if (( data->GetLong(AMa_CHMMKR_HN_WHAT) == AMa_CHMMKR_HN_OFF) ||
                ( data->GetLong(AMa_CHMMKR_HN_WHAT) == AMa_CHMMKR_HN_P_ONLY) ) {
                bc = description->GetParameterI(DescLevel(AMa_CHMMKR_HN_PRIORITY), ar);
                if ( bc) {
                    bc->SetBool(DESC_HIDE, TRUE);
                }
            }
            if ( data->GetLong(AMa_CHMMKR_COMP_SEC_MO) == AMa_CHMMKR_MODE_AMaPARAL) {
                bc = description->GetParameterI(DescLevel(AMa_CHMMKR_GROUP_STANDARD), ar);
                if ( bc) {
                    bc->SetBool(DESC_HIDE, TRUE);
                }
                if ( !data->GetBool(AMa_CHMMKR_COMP_CONN)) {
                    bc = description->GetParameterI(DescLevel(AMa_CHMMKR_COMP_CO_THR), ar);
                    if ( bc) {
                        bc->SetBool(DESC_HIDE, TRUE);
                    }
                }
            } else {
                bc = description->GetParameterI(DescLevel(AMa_CHMMKR_GROUP_DBL_P), ar);
                if ( bc) {
                    bc->SetBool(DESC_HIDE, TRUE);
                }
                if ( data->GetLong(AMa_CHMMKR_COMP_SEC_MO) != AMa_CHMMKR_MODE_USER) {
                    bc = description->GetParameterI(DescLevel(AMa_CHMMKR_GROUP_USER), ar);
                    if ( bc) {
                        bc->SetBool(DESC_HIDE, TRUE);
                    }
                }
            }
            break;
        case AMa_CHMMKR_MODE_USER:
            bc = description->GetParameterI(DescLevel(AMa_CHMMKR_GROUP_PARALL), ar);
            if ( bc) {
                bc->SetBool(DESC_HIDE, TRUE);
            }
            bc = description->GetParameterI(DescLevel(AMa_CHMMKR_GROUP_COMP), ar);
            if ( bc) {
                bc->SetBool(DESC_HIDE, TRUE);
            }
            break;
        default:
            bc = description->GetParameterI(DescLevel(AMa_CHMMKR_GROUP_PARALL), ar);
            if ( bc) {
                bc->SetBool(DESC_HIDE, TRUE);
            }
            bc = description->GetParameterI(DescLevel(AMa_CHMMKR_GROUP_COMP), ar);
            if ( bc) {
                bc->SetBool(DESC_HIDE, TRUE);
            }
            bc = description->GetParameterI(DescLevel(AMa_CHMMKR_GROUP_USER), ar);
            if ( bc) {
                bc->SetBool(DESC_HIDE, TRUE);
            }
    } // switch

    flags |= DESCFLAGS_DESC_LOADED;
    return ObjectData::GetDDescription(node, description, flags);
} // GetDDescription

// ----------GetVirtualObjects-------------------------------------------------------------------------------
BaseObject * AMaChamMaker::GetVirtualObjects(BaseObject * gener, HierarchyHelp * hh) {
    BaseObject * orig = gener->GetDown();

    if ( !orig) {
        return NULL;
    }

    Bool dirty = FALSE;
    BaseObject * main = NULL;
    BaseObject * res;
    res = gener->GetAndCheckHierarchyClone(hh, orig, HIERARCHYCLONEFLAGS_ASPOLY, &dirty, NULL, data->GetBool(AMa_CHMMKR_ALL_CHILDREN));
    if ( !dirty) {
        return res;
    }
    if ( !res) {
        return NULL;
    }
    String s_hierarchyPath("");

    if ( data->GetBool(AMa_CHMMKR_ALL_CHILDREN)) {
        if ( !Recurse_All_Childs(gener, hh->GetThread(), res, s_hierarchyPath)) {
            blDelete(res);
            return NULL;
        }
    } else {
        main = BaseObject::Alloc(Onull);
        if ( !Recurse_First_Child(gener, hh->GetThread(), res, main, s_hierarchyPath)) {
            blDelete(res);
            blDelete(main);
            return NULL;
        }
        blDelete(res);
        return main;
    }

    return res;
}

// ----------Recurse_All_Childs-------------------------------------------------------------------------------------
Bool AMaChamMaker::Recurse_All_Childs(BaseObject * gener, BaseThread * bt, BaseObject * op, String s_hierarPath) {
    if ( op->IsInstanceOf(Opolygon)) {
        BaseObject * realObj = gener;
        for ( LONG i = 0; i < s_hierarPath.GetLength(); i++) {
            switch ( s_hierarPath[i]) {
                case 'd':
                    realObj = realObj->GetDown();
                    break;
                case 'n':
                    realObj = realObj->GetNext();
                    break;
            }
            if ( !realObj) {
                break;
            }
        }
        chk = ModifyObj(gener, op, realObj);
        if ( !chk) {
            return FALSE;
        }
    }
    s_hierarPath += "d";
    for ( op = op->GetDown(); op; op = op->GetNext()) {
        chk = Recurse_All_Childs(gener, bt, op, s_hierarPath);
        if ( !chk) {
            return FALSE;
        }
        s_hierarPath += "n";
    }
    return ( !bt || !bt->TestBreak());        // check for user break
}

// ----------Recurse_First_Child-------------------------------------------------------------------------------------
Bool AMaChamMaker::Recurse_First_Child(BaseObject * gener, BaseThread * bt, BaseObject * op,
                                       BaseObject * main, String s_hierarPath) {
    BaseObject * op_copy;
    Matrix matrG;

    if ( op->IsInstanceOf(Opolygon)) {
        BaseObject * realObj = gener->GetDown();
        for ( LONG i = 0; i < s_hierarPath.GetLength(); i++) {
            switch ( s_hierarPath[i]) {
                case 'd':
                    realObj = realObj->GetDown();
                    break;
                case 'n':
                    realObj = realObj->GetNext();
                    break;
            }
            if ( !realObj) {
                break;
            }
        }
        chk = ModifyObj(gener, op, realObj);
        if ( !chk) {
            return FALSE;
        }
        op_copy = (BaseObject *)op->GetClone(COPYFLAGS_NO_HIERARCHY | COPYFLAGS_NO_ANIMATION, NULL);
        op_copy->InsertUnderLast(main);
        op_copy->SetMg(op->GetMg());
    }
    s_hierarPath += "d";
    for ( op = op->GetDown(); op; op = op->GetNext()) {
        chk = Recurse_First_Child(gener, bt, op, main, s_hierarPath);
        if ( !chk) {
            return FALSE;
        }
        s_hierarPath += "n";
    }

    return ( !bt || !bt->TestBreak());        // check for user break
}

// ----------ModifyObj--------------------------------------------------------------------------------------
Bool AMaChamMaker::ModifyObj(BaseObject * mod, BaseObject * op, BaseObject * realObj) {
    NbrInited = false;
    krnlInited = false;
    obj = ToPoly(op);
    selE = obj->GetEdgeS();
    data = mod->GetDataInstance();
    const CPolygon * ObjPlgns = obj->GetPolygonR();

    switch ( data->GetLong(AMa_CHMMKR_SEL_MODE)) {
        case AMa_CHMMKR_SELMO_NSSET:
        {
            SelectionTag * TagEdgSel = static_cast<SelectionTag *>(
                    obj->GetTag(Tedgeselection, data->GetLong(AMa_CHMMKR_N_TH_SS) - 1));
            if ( !TagEdgSel) {
                return TRUE;
            } else {
                TagEdgSel->GetBaseSelect()->CopyTo(selE);
            }
            break;
        }
        case AMa_CHMMKR_SELMO_LIVE:
            break;
        case AMa_CHMMKR_SELMO_OPEN:
        {
            selE->DeselectAll();
            nbr.Init(obj->GetPointCount(), ObjPlgns, obj->GetPolygonCount(), NULL);
            NbrInited = true;
            SelectOpenEdges(obj, &nbr, selE);
            break;
        }
        case AMa_CHMMKR_SELMO_PHO:
        {
            BaseSelect * selPho = obj->GetPhongBreak();
            selPho->CopyTo(selE);
            if ( data->GetBool(AMa_CHMMKR_AND_OPEN_EDG)) {
                nbr.Init(obj->GetPointCount(), ObjPlgns, obj->GetPolygonCount(), NULL);
                NbrInited = true;
                SelectOpenEdges(obj, &nbr, selE);
            }
            break;
        }
        case AMa_CHMMKR_SELMO_ALL:
        {
            selE->SelectAll(0, obj->GetPolygonCount() * 4 - 1);
            break;
        }
        case AMa_CHMMKR_SELMO_CHOOSE:
        {
            selE->DeselectAll();
            SelectionTag * TagEdgSel;
            GeData dataTMP;
            mod->GetParameter(DescID(AMa_CHMMKR_IN_EXCL), dataTMP, DESCFLAGS_GET_0);
            InExcludeData * inclListData =
                static_cast<InExcludeData *>( dataTMP.GetCustomDataType(CUSTOMDATATYPE_INEXCLUDE_LIST));
            if ( inclListData) {
                for ( LONG i = 0; i < inclListData->GetObjectCount(); i++) {
                    TagEdgSel = static_cast<SelectionTag *>( inclListData->ObjectFromIndex(op->GetDocument(), i));
                    if ( TagEdgSel && ( TagEdgSel->GetObject() == realObj) ) {
                        selE->Merge(TagEdgSel->GetBaseSelect());
                    }
                }
            }
            break;
        }
        case AMa_CHMMKR_SELMO_THRESH:
        {
            selE->DeselectAll();
            nbr.Init(obj->GetPointCount(), ObjPlgns, obj->GetPolygonCount(), NULL);
            NbrInited = true;
            krnl = Modeling::Alloc();
            if ( !krnl) {
                return false;
            }
            chk = krnl->InitObject(obj);
            if ( !chk) {
                return false;
            }
            krnlInited = true;
            SelectEdgesWithThrAngle(obj, &nbr, krnl, selE,
                                    data->GetReal(AMa_CHMMKR_SELECT_THRESH), data->GetBool(AMa_CHMMKR_AND_OPEN_EDG));
            break;
        }
    } // switch

    if ( selE->GetCount() == 0) {
        if ( NbrInited) {
            nbr.Flush();
            NbrInited = false;
        }
        if ( krnlInited) {
            Modeling::Free(krnl);
            krnlInited = false;
        }
        return TRUE;
    }

    Bool Need_C4D_Bevel = true;
    Bool CompMode;
    LONG C4D_Bevel_Mode;
    if ( data->GetLong(AMa_CHMMKR_SHAPE_MODE) == AMa_CHMMKR_MODE_COMPO) {
        C4D_Bevel_Mode = data->GetLong(AMa_CHMMKR_COMP_SEC_MO);
        CompMode = true;
    } else {
        C4D_Bevel_Mode = data->GetLong(AMa_CHMMKR_SHAPE_MODE);
        CompMode = false;
    }

    switch ( C4D_Bevel_Mode) {
        case AMa_CHMMKR_MODE_LINEAR:
            C4D_Bevel_Mode = MDATA_BEVEL_MODE_LINEAR;
            break;
        case AMa_CHMMKR_MODE_CONVEX:
            C4D_Bevel_Mode = MDATA_BEVEL_MODE_OUTER_CIRCLE;
            break;
        case AMa_CHMMKR_MODE_CONCAVE:
            C4D_Bevel_Mode = MDATA_BEVEL_MODE_INNER_CIRCLE;
            break;
        case AMa_CHMMKR_MODE_BEZIER:
            C4D_Bevel_Mode = MDATA_BEVEL_MODE_BEZIER;
            break;
        case AMa_CHMMKR_MODE_USER:
            C4D_Bevel_Mode = MDATA_BEVEL_MODE_USER;
            break;
        case AMa_CHMMKR_MODE_AMaPARAL:
            Need_C4D_Bevel = false;
            break;
    }

    if ( CompMode) {
        chk = MakeChamf_In_AMa_PARALLEL_Mode(true, !Need_C4D_Bevel, true);         // ////////   ! ! ! !
        if ( !chk) {
            return FALSE;
        }
    }
    if ( !Need_C4D_Bevel) {
        chk = MakeChamf_In_AMa_PARALLEL_Mode(CompMode, CompMode, false);           // ////////   ! ! ! !
        if ( !chk) {
            return FALSE;
        }
    }

    if ( NbrInited) {
        nbr.Flush();
        NbrInited = false;
    }
    if ( krnlInited) {
        Modeling::Free(krnl);
        krnlInited = false;
    }

    if ( Need_C4D_Bevel) {
        BaseContainer bc;
#if API_VERSION < 15000
        bc.SetReal(MDATA_BEVEL_OFFSET2, data->GetReal(AMa_CHMMKR_RAD_a));
        bc.SetReal(MDATA_BEVEL_VARIANCE2, data->GetReal(AMa_CHMMKR_VARIANCE));
        bc.SetLong(MDATA_BEVEL_SUBDIVISION, data->GetLong(AMa_CHMMKR_SUBDIVISION));
        bc.SetBool(MDATA_BEVEL_CREATENGONS, data->GetBool(AMa_CHMMKR_CREATENGONS));
        bc.SetLong(MDATA_BEVEL_MODE, C4D_Bevel_Mode);
        bc.SetData(MDATA_BEVEL_PATH, ( data->GetData(AMa_CHMMKR_PATH)));
        bc.SetReal(MDATA_BEVEL_OFFSET1, data->GetReal(AMa_CHMMKR_EXTRUSION));
        bc.SetReal(MDATA_BEVEL_VARIANCE1, data->GetReal(AMa_CHMMKR_EXTRU_VARI));
#else
		// TODO: Implement R15 Bevel Tool Settings
#endif
        ModelingCommandData cd;
        cd.doc = op->GetDocument();
        cd.bc = &bc;
        cd.op = obj;
        cd.mode = MODELINGCOMMANDMODE_EDGESELECTION;
        cd.arr = NULL;
        chk = SendModelingCommand(_ID_BEVELTOOL, cd);
        if ( !chk) {
            return FALSE;
        }
    }
    // GePrint( LongToString( sizeof( CHAR))) ;
    return TRUE;
} // ModifyObj

// -----------Register_AMa_Deformer--------------------------------------------------------------------
#define ID_AMa_CHAMFER_MAKER 1020439    // be sure to use a unique ID obtained from www.plugincafe.com

Bool Register_AMa_Deformer(void) {
    String name = GeLoadString(IDS_AMaChamferMAKER_MENU_NAME);

    if ( !name.Content()) {
        return TRUE;
    }
    // Bool RegisterObjectPlugin(LONG id, const String &str, LONG info, DataAllocator *g,
    //                           const String &description, BaseBitmap *icon, LONG disklevel);
    return RegisterObjectPlugin(ID_AMa_CHAMFER_MAKER, name, OBJECT_GENERATOR | OBJECT_INPUT,
                                AMaChamMaker::Alloc, "O_AMa_ChamferMaker", AutoBitmap("AMa_ChamferMaker.tif"), 0);
}

// -----------get_HN_tag--------------------------------------------------------------------

HNWeightTag * AMaChamMaker::get_HN_tag() {
    for ( BaseTag * tag = obj->GetFirstTag(); tag; tag = tag->GetNext()) {
        if ( tag->GetType() == 1007579) {
            return ( (HNWeightTag *)tag);
        }
    }
    return NULL;
}

// oooooooooooooooooooooooooooooooo AMa_PARALLEL ooooooooooooooooooooooooooooooooooooooooooooooo
// WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW

struct EdgeDataStruct {
    LONG pA, pB, num;
    EdgeDataStruct * wing_AL, * wing_BL, * wing_AR, * wing_BR;
    EdgeDataStruct * secWing_AL, * secWing_BL, * secWing_AR, * secWing_BR;
    LONG secWing_AL_pnt, secWing_AR_pnt, secWing_BL_pnt, secWing_BR_pnt;
    EdgeDataStruct * wingOfWing_AL1, * wingOfWing_AL2, * wingOfWing_BL1, * wingOfWing_BL2;
    EdgeDataStruct * wingOfWing_AR1, * wingOfWing_AR2, * wingOfWing_BR1, * wingOfWing_BR2;
    LONG wingPntAL, wingPntBL, wingPntAR, wingPntBR;
    Bool treated, open;
    LONG pA_ins, pB_ins;
    LONG wingAL_ins, wingBL_ins, wingAR_ins, wingBR_ins;
    LONG outlAL_ins, outlBL_ins, outlAR_ins, outlBR_ins;
    LONG fanCntAL, fanCntBL, fanCntAR, fanCntBR;
    EdgeDataStruct * fanEndAL, * fanEndBL, * fanEndAR, * fanEndBR;
    Bool fanDisableAL, fanDisableAR, fanDisableBL, fanDisableBR;
    CHAR fanConterside[4];
    Real HN_weight[4], HN_weight_edg;
    Vector vector;
    CHAR connCheked[4];
    Bool closeForWeld[4];

    inline LONG p(CHAR w) {
        /*if( w % 2)
                return pB ;
           else
                return pA ;*/
        return ( (&pA)[ w % 2]);
    };
    inline EdgeDataStruct * wing(CHAR w) {
        /*switch( w)
           {
                case 0: return wing_AL ;
                case 1: return wing_BL ;
                case 2: return wing_AR ;
                case 3: return wing_BR ;
           }*/
        return ( (&wing_AL)[w]);
    };
    inline EdgeDataStruct * secWing(CHAR w) {
        /*switch( w)
           {
                case 0: return secWing_AL ;
                case 1: return secWing_BL ;
                case 2: return secWing_AR ;
                case 3: return secWing_BR ;
           }*/
        return ( (&secWing_AL)[w]);
    };
    inline LONG p_ins(CHAR w) {
        /*if( w % 2)
                return pB_ins ;
           else
                return pA_ins ;*/
        return ( (&pA_ins)[ w % 2]);
    };
    inline LONG wing_ins(CHAR w) {
        /*switch( w)
           {
                case 0: return wingAL_ins ;
                case 1: return wingBL_ins ;
                case 2: return wingAR_ins ;
                case 3: return wingBR_ins ;
           }*/
        return ( (&wingAL_ins)[w]);
    };
    inline LONG outl_ins(CHAR w) {
        /*switch( w)
           {
                case 0: return outlAL_ins ;
                case 1: return outlBL_ins ;
                case 2: return outlAR_ins ;
                case 3: return outlBR_ins ;
           }*/
        return ( (&outlAL_ins)[w]);
    };
    CHAR GetCounterpart(LONG pnt_head, LONG pnt_tail) {
        if ( pnt_head == pA) {
            if ( pnt_tail == wingPntAL) {
                return 4;
            }
            if ( pnt_tail == wingPntAR) {
                return 6;
            }
            return FALSE;
        }
        if ( pnt_head == pB) {
            if ( pnt_tail == wingPntBL) {
                return 5;
            }
            if ( pnt_tail == wingPntBR) {
                return 7;
            }
            return FALSE;
        }
        return FALSE;
    };
    LONG GetAdjacentInserted(LONG pnt_end, LONG pnt_wing) {
        if ( pnt_end == pA) {
            if ( pnt_wing == wingPntAL) {
                return wingAL_ins;
            }
            if ( pnt_wing == wingPntAR) {
                return wingAR_ins;
            }
            return -1000000;
        }
        if ( pnt_end == pB) {
            if ( pnt_wing == wingPntBL) {
                return wingBL_ins;
            }
            if ( pnt_wing == wingPntBR) {
                return wingBR_ins;
            }
            return -1000000;
        }
        return -1000000;
    };
    LONG Get_AtoB_inserted(LONG pnt_end, LONG pnt_wing) {
        if ( pnt_end == pA) {
            if ( pnt_wing == wingPntAL) {
                return wingBL_ins;
            }
            if ( pnt_wing == wingPntAR) {
                return wingBR_ins;
            }
            return -1000000;
        }
        if ( pnt_end == pB) {
            if ( pnt_wing == wingPntBL) {
                return wingAL_ins;
            }
            if ( pnt_wing == wingPntBR) {
                return wingAR_ins;
            }
            return -1000000;
        }
        return -1000000;
    };
    CHAR Turn_Fan_Off(LONG pab, LONG pwing) {
        if ( pab == pA) {
            if ( pwing == wingPntAL) {
                fanDisableAL = true;
                return 0;
            }
            if ( pwing == wingPntAR) {
                fanDisableAR = true;
                return 2;
            }
        }
        if ( pab == pB) {
            if ( pwing == wingPntBL) {
                fanDisableBL = true;
                return 1;
            }
            if ( pwing == wingPntBR) {
                fanDisableBR = true;
                return 3;
            }
        }
        return 0;
    };
};

struct ConnectOutlinesChain_Struct {
    EdgeDataStruct * edgeStart, * edgeEnd;
    CHAR ab_Start, ab_End;
    ConnectOutlinesChain_Struct * next;
    LONG pnt1, pnt2;
};

struct ConnectOutlinesDot_Struct {
    LONG pnt;
    EdgeDataStruct * edge[2];
    CHAR edgeSide[2];
    ConnectOutlinesChain_Struct * chain;
};

// WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW
// -----------------MakeChamf_In_AMa_PARALLEL_Mode----------------------------------------------------
// WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW

Bool AMaChamMaker::MakeChamf_In_AMa_PARALLEL_Mode(Bool CompositeModeOn, Bool DoubleParallel, Bool CompositeFirstPass) {
    Real ConnectThresh, ChamfRadius, Weld, FistPassOfDouble = false;
    Bool CloseFans, ProlongOutlines, ConnectOutlines;
    LONG OrigPntCnt = obj->GetPointCount();
    LONG OrigPlgnCnt = obj->GetPolygonCount();

    LONG longUseHN = data->GetLong(AMa_CHMMKR_HN_WHAT);
    Bool HN_priority, useHN = ( longUseHN != AMa_CHMMKR_HN_OFF);
    Bool delHN_Tag = data->GetBool(AMa_CHMMKR_HN_RESET);
    Bool HN_useEdges = false, HN_usePoints = false;
    HNWeightTag * NH_Tag = NULL;
    HNData HN_data;
    Real * HN_plg_weights;

    if ( useHN || delHN_Tag) {
        NH_Tag = get_HN_tag();
        if ( !NH_Tag) {
            useHN = false;
            delHN_Tag = false;
        }
    }
    if ( useHN) {
        if ( !NH_Tag->GetTagData(&HN_data)) {
            useHN = false;
        } else if (( *HN_data.points < OrigPntCnt) || ( *HN_data.polys < OrigPlgnCnt) ) {
            useHN = false;
            GePrint("HyperNURBS Tag has not enough elements");
        } else {
            HN_plg_weights = *( (Real **)HN_data.polyweight);
            HN_priority = ( data->GetLong(AMa_CHMMKR_HN_PRIORITY) == AMa_CHMMKR_HN_BIG);
            if (( longUseHN == AMa_CHMMKR_HN_E_ONLY) || ( longUseHN == AMa_CHMMKR_HN_BOTH) ) {
                HN_useEdges = true;
            }
            if (( longUseHN == AMa_CHMMKR_HN_P_ONLY) || ( longUseHN == AMa_CHMMKR_HN_BOTH) ) {
                HN_usePoints = true;
            }
        }
    }

    if ( DoubleParallel && !CompositeFirstPass) {
        CloseFans = data->GetBool(AMa_CHMMKR_COMP_CLO_F);
        ProlongOutlines = data->GetBool(AMa_CHMMKR_COMP_PR_O);
        ConnectOutlines = data->GetBool(AMa_CHMMKR_COMP_CONN);
        ConnectThresh = Cos(Rad(data->GetReal(AMa_CHMMKR_COMP_CO_THR)));
        Weld = FALSE;
        ChamfRadius = data->GetReal(AMa_CHMMKR_RAD_c);
    } else {
        CloseFans = data->GetBool(AMa_CHMMKR_CLOSE_FANS);
        ProlongOutlines = data->GetBool(AMa_CHMMKR_PROL_OUTL);
        ConnectOutlines = data->GetBool(AMa_CHMMKR_CONNECT_OUTL);
        ConnectThresh = Cos(Rad(data->GetReal(AMa_CHMMKR_CONN_THRESH)));
        Weld = data->GetBool(AMa_CHMMKR_WELD);
        if ( CompositeModeOn) {
            if ( DoubleParallel) {
                ChamfRadius = data->GetReal(AMa_CHMMKR_RAD_PERC) * data->GetReal(AMa_CHMMKR_RAD_c);
                FistPassOfDouble = true;
            } else {
                ChamfRadius = data->GetReal(AMa_CHMMKR_RAD_PERC) * data->GetReal(AMa_CHMMKR_RAD_a);
            }
        } else {
            ChamfRadius = data->GetReal(AMa_CHMMKR_RAD_p);
        }
    }

    if ( !krnlInited) {
        krnl = Modeling::Alloc();
        if ( !krnl) {
            return false;
        }
        chk = krnl->InitObject(obj);
        if ( !chk) {
            return false;
        }
        krnlInited = true;
    }

    const CPolygon * ObjPlgns = obj->GetPolygonR();
    const Vector * OrigPoints = obj->GetPointR();
    if ( !NbrInited) {
        nbr.Init(OrigPntCnt, ObjPlgns, OrigPlgnCnt, NULL);
        NbrInited = true;
    }
    LONG i, e, seg_start, seg_end;
    LONG totalEdgeCnt = OrigPlgnCnt * 4;
    CHAR s;
    EdgeDataStruct ** EdgeData = (EdgeDataStruct **)GeAlloc(sizeof(EdgeDataStruct *) * totalEdgeCnt);
    if ( !EdgeData) {
        return false;
    }
    LONG * indEDa = (LONG *)GeAlloc(sizeof(LONG *) * totalEdgeCnt);
    if ( !indEDa) {
        return false;
    }
    LONG ChargedEDaCount = 0;
    i = 0;
    while ( selE->GetRange(i, MAXLONGl, &seg_start, &seg_end)) {
        i++;
        for ( e = seg_start; e <= seg_end; e++) {
            if ( !IsEdgeDoubled(&nbr, e)) {
                Bool valid;
                EdgeData[e] = (EdgeDataStruct *)GeAlloc(sizeof(EdgeDataStruct));
                if ( !EdgeData[e]) {
                    return false;
                }
                EdgeDataStruct * ed = EdgeData[e];
                Get_x4_edge_points(ObjPlgns, e, ed->pA, ed->pB);
                chk = Get_H_topol_points(krnl, obj, ed->pA, ed->pB, valid, ed->wingPntAL,
                                         ed->wingPntAR, ed->open, ed->wingPntBL, ed->wingPntBR);
                if ( !chk) {
                    return false;
                }

                if ( !valid || ((( ed->wingPntAL == ed->wingPntAR) || ( ed->wingPntBL == ed->wingPntBR) ) && !ed->open)) {
                    GeFree(EdgeData[e]);
                    continue;
                }

                if ( HN_useEdges) {
                    /*LONG E_In_P = e % 4 ;
                       LONG P_of_E = ( e - E_In_P) / 4 ;
                       switch( E_In_P)
                       {
                            case 0: ed->HN_edg = (*HN_data.polyweight)[P_of_E].a ; break ;
                            case 1: ed->HN_edg = (*HN_data.polyweight)[P_of_E].b ; break ;
                            case 2: ed->HN_edg = (*HN_data.polyweight)[P_of_E].c ; break ;
                            case 3: ed->HN_edg = (*HN_data.polyweight)[P_of_E].d ; break ;
                       }*/
                    ed->HN_weight_edg = 1.0 + HN_plg_weights[e];
                }
                ed->pA_ins = ed->pA;
                ed->pB_ins = ed->pB;
                ed->num = ChargedEDaCount;
                if ( ConnectOutlines) {
                    ed->vector = !( OrigPoints[ed->pA] - OrigPoints[ed->pB]);
                }

                indEDa[ChargedEDaCount] = e;
                ChargedEDaCount++;
            }
            if ( HN_useEdges && !FistPassOfDouble) {
                HN_plg_weights[e] = 0;
            }
        }
    }

    // /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // ////////////////////////----------------------------------------------------  fill adjacent edges data (wings)
    for ( i = 0; i < ChargedEDaCount; i++) {
        EdgeDataStruct * ed = EdgeData[indEDa[i]];
        LONG e_tmp;
        // ----------------------------------------------------------------------
        e_tmp = GetEdgeFromPoints(ObjPlgns, &nbr, ed->pA, ed->wingPntAL);
        if ( e_tmp < 0) {
            return false;
        }
        ed->wing_AL = EdgeData[e_tmp];
        e_tmp = GetEdgeFromPoints(ObjPlgns, &nbr, ed->pB, ed->wingPntBL);
        if ( e_tmp < 0) {
            return false;
        }
        ed->wing_BL = EdgeData[e_tmp];
        // ----------------------------------------------------------------------------------- second wings A
        LONG p3;
        Bool finded;
        chk = GetNextPointInFan(krnl, obj, ed->pA, ed->pB, ed->wingPntAL, p3, finded);
        if ( !chk) {
            return false;
        }
        if ( finded) {
            e_tmp = GetEdgeFromPoints(ObjPlgns, &nbr, ed->pA, p3);
            if ( e_tmp == -1) {
                return false;
            }
            ed->secWing_AL_pnt = p3;
            ed->secWing_AL = EdgeData[e_tmp];
        } else {
            ed->secWing_AL_pnt = -1;
        }
        // ------------------------------------------------------------------------------------ wings of wings A
        LONG pww;
        chk = findNextPointAlongNgn(krnl, obj, ed->pB, ed->pA, ed->wingPntAL, pww);
        if ( !chk) {
            return false;
        }
        e_tmp = GetEdgeFromPoints(ObjPlgns, &nbr, ed->wingPntAL, pww);
        if ( e_tmp < 0) {
            return false;
        }
        ed->wingOfWing_AL1 = EdgeData[e_tmp];
        if ( finded) {
            chk = findNextPointAlongNgn(krnl, obj, p3, ed->pA, ed->wingPntAL, pww);
            if ( !chk) {
                return false;
            }
            e_tmp = GetEdgeFromPoints(ObjPlgns, &nbr, ed->wingPntAL, pww);
            if ( e_tmp == -1) {
                return false;
            }
            ed->wingOfWing_AL2 = EdgeData[e_tmp];
        }
        // ----------------------------------------------------------------------------------- second wings B
        chk = GetNextPointInFan(krnl, obj, ed->pB, ed->pA, ed->wingPntBL, p3, finded);
        if ( !chk) {
            return false;
        }
        if ( finded) {
            e_tmp = GetEdgeFromPoints(ObjPlgns, &nbr, ed->pB, p3);
            if ( e_tmp == -1) {
                return false;
            }
            ed->secWing_BL_pnt = p3;
            ed->secWing_BL = EdgeData[e_tmp];
        } else {
            ed->secWing_BL_pnt = -1;
        }
        // ------------------------------------------------------------------------------------ wings of wings B
        chk = findNextPointAlongNgn(krnl, obj, ed->pA, ed->pB, ed->wingPntBL, pww);
        if ( !chk) {
            return false;
        }
        e_tmp = GetEdgeFromPoints(ObjPlgns, &nbr, ed->wingPntBL, pww);
        if ( e_tmp < 0) {
            return false;
        }
        ed->wingOfWing_BL1 = EdgeData[e_tmp];
        if ( finded) {
            chk = findNextPointAlongNgn(krnl, obj, p3, ed->pB, ed->wingPntBL, pww);
            if ( !chk) {
                return false;
            }
            e_tmp = GetEdgeFromPoints(ObjPlgns, &nbr, ed->wingPntBL, pww);
            if ( e_tmp == -1) {
                return false;
            }
            ed->wingOfWing_BL2 = EdgeData[e_tmp];
        }

        // ------------------------------------------------------------------------------------    IF OPEN
        // ////////////////////////----------------------------------------------------  fill adjacent edges data (wings)
        if ( ed->open) {
            continue;
        }

        e_tmp = GetEdgeFromPoints(ObjPlgns, &nbr, ed->pA, ed->wingPntAR);
        if ( e_tmp < 0) {
            return false;
        }
        ed->wing_AR = EdgeData[e_tmp];

        e_tmp = GetEdgeFromPoints(ObjPlgns, &nbr, ed->pB, ed->wingPntBR);
        if ( e_tmp < 0) {
            return false;
        }
        ed->wing_BR = EdgeData[e_tmp];
        // ---------------------------------------------------------------------------------- second wings A
        chk = GetNextPointInFan(krnl, obj, ed->pA, ed->pB, ed->wingPntAR, p3, finded);
        if ( !chk) {
            return false;
        }
        if ( finded) {
            e_tmp = GetEdgeFromPoints(ObjPlgns, &nbr, ed->pA, p3);
            if ( e_tmp == -1) {
                return false;
            }
            ed->secWing_AR_pnt = p3;
            ed->secWing_AR = EdgeData[e_tmp];
        } else {
            ed->secWing_AR_pnt = -1;
        }
        // ------------------------------------------------------------------------------------ wings of wings A
        chk = findNextPointAlongNgn(krnl, obj, ed->pB, ed->pA, ed->wingPntAR, pww);
        if ( !chk) {
            return false;
        }
        e_tmp = GetEdgeFromPoints(ObjPlgns, &nbr, ed->wingPntAR, pww);
        if ( e_tmp < 0) {
            return false;
        }
        ed->wingOfWing_AR1 = EdgeData[e_tmp];
        if ( finded) {
            chk = findNextPointAlongNgn(krnl, obj, p3, ed->pA, ed->wingPntAR, pww);
            if ( !chk) {
                return false;
            }
            e_tmp = GetEdgeFromPoints(ObjPlgns, &nbr, ed->wingPntAR, pww);
            if ( e_tmp == -1) {
                return false;
            }
            ed->wingOfWing_AR2 = EdgeData[e_tmp];
        }
        // ----------------------------------------------------------------------------------- second wings B
        chk = GetNextPointInFan(krnl, obj, ed->pB, ed->pA, ed->wingPntBR, p3, finded);
        if ( !chk) {
            return false;
        }
        if ( finded) {
            e_tmp = GetEdgeFromPoints(ObjPlgns, &nbr, ed->pB, p3);
            if ( e_tmp == -1) {
                return false;
            }
            ed->secWing_BR_pnt = p3;
            ed->secWing_BR = EdgeData[e_tmp];
        } else {
            ed->secWing_BR_pnt = -1;
        }
        // ------------------------------------------------------------------------------------ wings of wings B
        chk = findNextPointAlongNgn(krnl, obj, ed->pA, ed->pB, ed->wingPntBR, pww);
        if ( !chk) {
            return false;
        }
        e_tmp = GetEdgeFromPoints(ObjPlgns, &nbr, ed->wingPntBR, pww);
        if ( e_tmp < 0) {
            return false;
        }
        ed->wingOfWing_BR1 = EdgeData[e_tmp];
        if ( finded) {
            chk = findNextPointAlongNgn(krnl, obj, p3, ed->pB, ed->wingPntBR, pww);
            if ( !chk) {
                return false;
            }
            e_tmp = GetEdgeFromPoints(ObjPlgns, &nbr, ed->wingPntBR, pww);
            if ( e_tmp == -1) {
                return false;
            }
            ed->wingOfWing_BR2 = EdgeData[e_tmp];
        }
    }

    // /////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // ////////////////////////-------------------------------/////////////////////////////////   fill fan data
    for ( i = 0; i < ChargedEDaCount; i++) {
        if ( !CloseFans) {
            break;
        }

        EdgeDataStruct * ed = EdgeData[indEDa[i]];
        // ----------------------------------------------------------------------
        LONG p1, p2, p3, e_tmp;
        Bool finded;
        // -------------------------------------------------------------- AL
        if ( !ed->wing_AL && !ed->secWing_AL && ( ed->secWing_AL_pnt != -1) && !ed->fanDisableAL) {
            p1 = ed->wingPntAL;
            p2 = ed->secWing_AL_pnt;
            while ( true) {
                ed->fanCntAL++;
                chk = GetNextPointInFan(krnl, obj, ed->pA, p1, p2, p3, finded);
                if ( !chk) {
                    return false;
                }
                if ( !finded || ( p3 == ed->pB) ) {
                    ed->fanCntAL = 0;
                    break;
                } else {
                    e_tmp = GetEdgeFromPoints(ObjPlgns, &nbr, ed->pA, p3);
                    if ( e_tmp < 0) {
                        return false;
                    }
                    if ( EdgeData[e_tmp]) {
                        ed->fanEndAL = EdgeData[e_tmp];
                        ed->fanConterside[0] = EdgeData[e_tmp]->Turn_Fan_Off(ed->pA, p2);
                        break;
                    }
                }
                p1 = p2;
                p2 = p3;
            }
        }
        // ------------------------------------------------------------------------- BL
        if ( !ed->wing_BL && !ed->secWing_BL && ( ed->secWing_BL_pnt != -1) && !ed->fanDisableBL) {
            p1 = ed->wingPntBL;
            p2 = ed->secWing_BL_pnt;
            while ( true) {
                ed->fanCntBL++;
                chk = GetNextPointInFan(krnl, obj, ed->pB, p1, p2, p3, finded);
                if ( !chk) {
                    return false;
                }
                if ( !finded || ( p3 == ed->pA) ) {
                    ed->fanCntBL = 0;
                    break;
                } else {
                    e_tmp = GetEdgeFromPoints(ObjPlgns, &nbr, ed->pB, p3);
                    if ( e_tmp < 0) {
                        return false;
                    }
                    if ( EdgeData[e_tmp]) {
                        ed->fanEndBL = EdgeData[e_tmp];
                        ed->fanConterside[1] = EdgeData[e_tmp]->Turn_Fan_Off(ed->pB, p2);
                        break;
                    }
                }
                p1 = p2;
                p2 = p3;
            }
        }

        if ( ed->open) {
            continue;
        }
        // ----------------------------------------------------------------------- AR
        if ( !ed->wing_AR && !ed->secWing_AR && ( ed->secWing_AR_pnt != -1) && !ed->fanDisableAR) {
            p1 = ed->wingPntAR;
            p2 = ed->secWing_AR_pnt;
            while ( true) {
                ed->fanCntAR++;
                chk = GetNextPointInFan(krnl, obj, ed->pA, p1, p2, p3, finded);
                if ( !chk) {
                    return false;
                }
                if ( !finded || ( p3 == ed->pB) ) {
                    ed->fanCntAR = 0;
                    break;
                } else {
                    e_tmp = GetEdgeFromPoints(ObjPlgns, &nbr, ed->pA, p3);
                    if ( e_tmp < 0) {
                        return false;
                    }
                    if ( EdgeData[e_tmp]) {
                        ed->fanEndAR = EdgeData[e_tmp];
                        ed->fanConterside[2] = EdgeData[e_tmp]->Turn_Fan_Off(ed->pA, p2);
                        break;
                    }
                }
                p1 = p2;
                p2 = p3;
            }
        }
        // ---------------------------------------------------------------------- BR
        if ( !ed->wing_BR && !ed->secWing_BR && ( ed->secWing_BR_pnt != -1) && !ed->fanDisableBR) {
            p1 = ed->wingPntBR;
            p2 = ed->secWing_BR_pnt;
            while ( true) {
                ed->fanCntBR++;
                chk = GetNextPointInFan(krnl, obj, ed->pB, p1, p2, p3, finded);
                if ( !chk) {
                    return false;
                }
                if ( !finded || ( p3 == ed->pA) ) {
                    ed->fanCntBR = 0;
                    break;
                } else {
                    e_tmp = GetEdgeFromPoints(ObjPlgns, &nbr, ed->pB, p3);
                    if ( e_tmp < 0) {
                        return false;
                    }
                    if ( EdgeData[e_tmp]) {
                        ed->fanEndBR = EdgeData[e_tmp];
                        ed->fanConterside[3] = EdgeData[e_tmp]->Turn_Fan_Off(ed->pB, p2);
                        break;
                    }
                }
                p1 = p2;
                p2 = p3;
            }
        }
    }

    // /////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // ////////////////////////-------------------------------////////////////   fill data for connect outlines
    ConnectOutlinesDot_Struct ** arrDotsForConnOutl = NULL;
    ConnectOutlinesChain_Struct * connOutlChainFirst = NULL, * connOutlChain = NULL, * connOutlChain_prev = NULL;
    LONG ConnOutlCnt = 0;
    Bool chainFilled = true;

    if ( ConnectOutlines) {
        arrDotsForConnOutl = (ConnectOutlinesDot_Struct **)GeAlloc(sizeof(ConnectOutlinesDot_Struct *) * OrigPntCnt);
        for ( i = 0; i < ChargedEDaCount; i++) {
            EdgeDataStruct * ed = EdgeData[indEDa[i]], * ed_start, * ed_end;
            CHAR ab_start, ab_end;
            // ----------------------------------------------------------------------
            if ( ed->open) {
                continue;
            }
            for ( CHAR w_lr = 0; w_lr < 3; w_lr += 2) {
                if ( CompositeFirstPass && chainFilled) {
                    connOutlChain = (ConnectOutlinesChain_Struct *)GeAlloc(sizeof(ConnectOutlinesChain_Struct));
                    if ( !connOutlChain) {
                        return false;
                    }
                    chainFilled = false;
                }
                ULONG quantityInLine = 0;
                Vector vectAverage = ed->vector;
                EdgeDataStruct * edc = ed, * edc_wing;
                CHAR wAnti;
                for ( CHAR w_ab = 0; w_ab < 2; w_ab++) {
                    CHAR w = w_ab + w_lr;
                    edc = ed;
                    while ( true) {
                        if ( edc->connCheked[w] & 1) {
                            break;
                        }
                        edc->connCheked[w] |= 1;

                        edc_wing = edc->wing(w);
                        if ( !edc_wing || edc_wing->open) {
                            break;
                        }

                        wAnti = edc_wing->GetCounterpart(edc->p(w), edc->p(w + 1));
                        if ( !wAnti) {
                            return false;
                        } else {
                            wAnti ^= 4;
                        }
                        if ( edc_wing->connCheked[ wAnti] & 1) {
                            break;
                        }
                        edc_wing->connCheked[wAnti] |= 1;

                        Real dotProduct = vectAverage * edc_wing->vector;

                        if ( dotProduct < 0) {
                            edc_wing->vector = -edc_wing->vector;
                            dotProduct = -dotProduct;
                        }
                        if ( dotProduct < ConnectThresh) {
                            break;
                        }

                        quantityInLine++;
                        vectAverage = !( vectAverage * quantityInLine + edc_wing->vector);
                        edc->connCheked[ w] |= 2;
                        edc->connCheked[ w ^ 2] |= 1;
                        edc_wing->connCheked[ wAnti] |= 2;
                        edc_wing->connCheked[ wAnti ^ 2] |= 1;

                        arrDotsForConnOutl[ ConnOutlCnt] =
                            (ConnectOutlinesDot_Struct *)GeAlloc(sizeof(ConnectOutlinesDot_Struct));
                        if ( !arrDotsForConnOutl[ ConnOutlCnt]) {
                            return false;
                        }
                        arrDotsForConnOutl[ ConnOutlCnt]->pnt = edc->p(w);
                        arrDotsForConnOutl[ ConnOutlCnt]->edge[0] = edc;
                        arrDotsForConnOutl[ ConnOutlCnt]->edge[1] = edc_wing;
                        arrDotsForConnOutl[ ConnOutlCnt]->edgeSide[0] = w;
                        arrDotsForConnOutl[ ConnOutlCnt]->edgeSide[1] = wAnti;
                        arrDotsForConnOutl[ ConnOutlCnt]->chain = connOutlChain;
                        ConnOutlCnt++;

                        edc = edc_wing;
                        w = wAnti ^ 1;
                    }
                    if ( !w_ab) {
                        ed_end = edc;
                        ab_end = w;
                    } else {
                        ed_start = edc;
                        ab_start = w;
                    }
                }
                if ( quantityInLine && CompositeFirstPass) {
                    connOutlChain->edgeStart = ed_start;
                    connOutlChain->edgeEnd = ed_end;
                    connOutlChain->ab_Start = ab_start;
                    connOutlChain->ab_End = ab_end;
                    if ( connOutlChain_prev) {
                        connOutlChain_prev->next = connOutlChain;
                    }
                    connOutlChain_prev = connOutlChain;
                    if ( !connOutlChainFirst) {
                        connOutlChainFirst = connOutlChain;
                    }
                    chainFilled = true;
                }
            }
        }
        if ( !chainFilled) {
            GeFree(connOutlChain);
        }
    }     // if ConnectOutlines

    // /////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // ////////////////////////-------------------------------////////////////   correct HN Weights
    if ( useHN) {
        if ( HN_useEdges) {
            for ( i = 0; i < ChargedEDaCount; i++) {
                EdgeDataStruct * ed = EdgeData[indEDa[i]];
                EdgeDataStruct ** pSecWing = &ed->secWing_AL;
                for ( s = 0; s < 4; s++) {
                    ed->HN_weight[s] = ed->HN_weight_edg;
                    if ( pSecWing[s] && (pSecWing[s]->HN_weight_edg != 1.0)) {
                        if ( ed->HN_weight[s] == 1.0) {
                            ed->HN_weight[s] = pSecWing[s]->HN_weight_edg;
                        } else {
                            if ( HN_priority) {
                                if ( ed->HN_weight[s] < pSecWing[s]->HN_weight_edg) {
                                    ed->HN_weight[s] = pSecWing[s]->HN_weight_edg;
                                }
                            } else
                            if ( ed->HN_weight[s] > pSecWing[s]->HN_weight_edg) {
                                ed->HN_weight[s] = pSecWing[s]->HN_weight_edg;
                            }
                        }
                    }
                }
            }
        }
        if ( longUseHN == AMa_CHMMKR_HN_P_ONLY) {
            for ( i = 0; i < ChargedEDaCount; i++) {
                EdgeDataStruct * ed = EdgeData[indEDa[i]];
                LONG * ppAB = &ed->pA;
                for ( s = 0; s < 4; s++) {
                    ed->HN_weight[s] = Abs(1.0 + ( *HN_data.pointweight)[ ppAB[ s % 2]]);
                }
            }
        }
        if ( longUseHN == AMa_CHMMKR_HN_BOTH) {
            for ( i = 0; i < ChargedEDaCount; i++) {
                EdgeDataStruct * ed = EdgeData[indEDa[i]];
                LONG * ppAB = &ed->pA;
                for ( s = 0; s < 4; s++) {
                    ed->HN_weight[s] *= Abs(1.0 + ( *HN_data.pointweight)[ ppAB[ s % 2]]);
                }
            }
        }

        if ( HN_usePoints && !delHN_Tag && !FistPassOfDouble) {
            for ( i = 0; i < ChargedEDaCount; i++) {
                EdgeDataStruct * ed = EdgeData[indEDa[i]];
                ( *HN_data.pointweight)[ed->pA] = 0;
                ( *HN_data.pointweight)[ed->pB] = 0;
            }
        }
    }     // if useHN

    if ( delHN_Tag) {
        NH_Tag->Remove();
        HNWeightTag ::Free(NH_Tag);
    } else if ( useHN) {
        *HN_data.changed = true;
    }

    // //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // ////////////////////////----------------------------------------------------------  MODELING  --------------------
    Vector vec1, vec2;
    LONG pn_wing_outl_far_ins, pn_plg_identif_1, pn_plg_identif_2, wingPnt_new_ins;
    Bool treatedWing_AL, treatedWing_BL, treatedWing_AR, treatedWing_BR;
    for ( i = 0; i < ChargedEDaCount; i++) {
        EdgeDataStruct * ed = EdgeData[indEDa[i]];
        if ( useHN) {
            for ( s = 0; s < 4; s++) {
                ed->HN_weight[s] = ChamfRadius * Abs(ed->HN_weight[s]);
            }
        } else {
            for ( s = 0; s < 4; s++) {
                ed->HN_weight[s] = ChamfRadius;
            }
        }
        treatedWing_AL = false;
        treatedWing_BL = false;
        treatedWing_AR = false;
        treatedWing_BR = false;

        // ----------------------------------------------------------  pA_ins  &  pB_ins   finding
        if (     ed->wing_AL && ed->wing_AL->treated) {
            ed->pA_ins = ed->wing_AL->GetAdjacentInserted(ed->pA, ed->pB);
            treatedWing_AL = true;
        }
        if ( ed->wing_AR && ed->wing_AR->treated) {
            ed->pA_ins = ed->wing_AR->GetAdjacentInserted(ed->pA, ed->pB);
            treatedWing_AR = true;
        }
        if ( ed->wing_BL && ed->wing_BL->treated) {
            ed->pB_ins = ed->wing_BL->GetAdjacentInserted(ed->pB, ed->pA);
            treatedWing_BL = true;
        }
        if ( ed->wing_BR && ed->wing_BR->treated) {
            ed->pB_ins = ed->wing_BR->GetAdjacentInserted(ed->pB, ed->pA);
            treatedWing_BR = true;
        }

        // ------------------------------------------------------------------------------------- AL
        if ( ed->secWing_AL && ed->secWing_AL->treated) {
            ed->wingAL_ins = ed->secWing_AL->GetAdjacentInserted(ed->pA, ed->wingPntAL);
        } else {
            if ( ed->wingOfWing_AL1 && ed->wingOfWing_AL1->treated) {
                wingPnt_new_ins = ed->wingOfWing_AL1->GetAdjacentInserted(ed->wingPntAL, ed->pA);
            } else if ( ed->wingOfWing_AL2 && ed->wingOfWing_AL2->treated) {
                wingPnt_new_ins = ed->wingOfWing_AL2->GetAdjacentInserted(ed->wingPntAL, ed->pA);
            } else {
                wingPnt_new_ins = ed->wingPntAL;
            }

            ed->wingAL_ins = SplitEdge_AbsDist(krnl, obj, ed->pA, wingPnt_new_ins, ed->HN_weight[0],
                                               ed->closeForWeld[0]);
            if ( !ed->wingAL_ins) {
                return false;
            }
        }
        ed->outlAL_ins = ed->wingAL_ins;

        if ( treatedWing_AL) {
            if ( ed->wingOfWing_AL1 && ed->wingOfWing_AL1->treated) {
                chk = findNextPointAlongNgn(krnl, obj, ed->wingAL_ins, ed->pA, ed->pA_ins, pn_wing_outl_far_ins);
                if ( !chk) {
                    return false;
                }
            } else {
                pn_wing_outl_far_ins = ed->wing_AL->Get_AtoB_inserted(ed->pA, ed->pB);
            }

            ed->outlAL_ins = SplitEdge_AbsDist(krnl, obj, ed->pA_ins, pn_wing_outl_far_ins, ed->HN_weight[0]);
            if ( !ed->outlAL_ins) {
                return false;
            }
            chk = ConnectPnts(krnl, obj, ed->wingAL_ins, ed->outlAL_ins, ed->pA_ins, ed->pA);
            if ( !chk) {
                return false;
            }
        }
        // ------------------------------------------------------------------------------------- BL
        if ( ed->secWing_BL && ed->secWing_BL->treated) {
            ed->wingBL_ins = ed->secWing_BL->GetAdjacentInserted(ed->pB, ed->wingPntBL);
        } else {
            if ( ed->wingOfWing_BL1 && ed->wingOfWing_BL1->treated) {
                wingPnt_new_ins = ed->wingOfWing_BL1->GetAdjacentInserted(ed->wingPntBL, ed->pB);
            } else if ( ed->wingOfWing_BL2 && ed->wingOfWing_BL2->treated) {
                wingPnt_new_ins = ed->wingOfWing_BL2->GetAdjacentInserted(ed->wingPntBL, ed->pB);
            } else {
                wingPnt_new_ins = ed->wingPntBL;
            }

            ed->wingBL_ins = SplitEdge_AbsDist(krnl, obj, ed->pB, wingPnt_new_ins, ed->HN_weight[1],
                                               ed->closeForWeld[1]);
            if ( !ed->wingBL_ins) {
                return false;
            }
        }
        ed->outlBL_ins = ed->wingBL_ins;

        if ( treatedWing_BL) {
            if ( ed->wingOfWing_BL1 && ed->wingOfWing_BL1->treated) {
                chk = findNextPointAlongNgn(krnl, obj, ed->wingBL_ins, ed->pB, ed->pB_ins, pn_wing_outl_far_ins);
                if ( !chk) {
                    return false;
                }
            } else {
                pn_wing_outl_far_ins = ed->wing_BL->Get_AtoB_inserted(ed->pB, ed->pA);
            }

            ed->outlBL_ins = SplitEdge_AbsDist(krnl, obj, ed->pB_ins, pn_wing_outl_far_ins, ed->HN_weight[1]);
            if ( !ed->outlBL_ins) {
                return false;
            }

            chk = ConnectPnts(krnl, obj, ed->wingBL_ins, ed->outlBL_ins, ed->pB_ins, ed->pB);
            if ( !chk) {
                return false;
            }
        }

        // ---------------------------------------------------------------------------- connect (long slice) L
        if ( treatedWing_AL) {
            pn_plg_identif_1 = ed->pA_ins;
            pn_plg_identif_2 = ed->pB_ins;
        } else if ( treatedWing_AR) {
            pn_plg_identif_1 = ed->pA;
            pn_plg_identif_2 = ed->pA_ins;
        } else {
            pn_plg_identif_1 = ed->pA_ins;
            pn_plg_identif_2 = ed->pB_ins;
        }
        chk = ConnectPnts(krnl, obj, ed->outlBL_ins, ed->outlAL_ins, pn_plg_identif_1, pn_plg_identif_2);
        if ( !chk) {
            return false;
        }

        if ( !ed->open) {
            // ------------------------------------------------------------------------------------- AR
            if ( ed->secWing_AR && ed->secWing_AR->treated) {
                ed->wingAR_ins = ed->secWing_AR->GetAdjacentInserted(ed->pA, ed->wingPntAR);
            } else {
                if ( ed->wingOfWing_AR1 && ed->wingOfWing_AR1->treated) {
                    wingPnt_new_ins = ed->wingOfWing_AR1->GetAdjacentInserted(ed->wingPntAR, ed->pA);
                } else if ( ed->wingOfWing_AR2 && ed->wingOfWing_AR2->treated) {
                    wingPnt_new_ins = ed->wingOfWing_AR2->GetAdjacentInserted(ed->wingPntAR, ed->pA);
                } else {
                    wingPnt_new_ins = ed->wingPntAR;
                }

                ed->wingAR_ins = SplitEdge_AbsDist(krnl, obj, ed->pA, wingPnt_new_ins, ed->HN_weight[2],
                                                   ed->closeForWeld[2]);
                if ( !ed->wingAR_ins) {
                    return false;
                }
            }
            ed->outlAR_ins = ed->wingAR_ins;

            if ( treatedWing_AR) {
                if ( ed->wingOfWing_AR1 && ed->wingOfWing_AR1->treated) {
                    chk = findNextPointAlongNgn(krnl, obj, ed->wingAR_ins, ed->pA, ed->pA_ins, pn_wing_outl_far_ins);
                    if ( !chk) {
                        return false;
                    }
                } else {
                    pn_wing_outl_far_ins = ed->wing_AR->Get_AtoB_inserted(ed->pA, ed->pB);
                }

                ed->outlAR_ins = SplitEdge_AbsDist(krnl, obj, ed->pA_ins, pn_wing_outl_far_ins, ed->HN_weight[2]);

                if ( !ed->outlAR_ins) {
                    return false;
                }
                chk = ConnectPnts(krnl, obj, ed->wingAR_ins, ed->outlAR_ins, ed->pA_ins, ed->pA);
                if ( !chk) {
                    return false;
                }
            }
            // ------------------------------------------------------------------------------------- BR
            if ( ed->secWing_BR && ed->secWing_BR->treated) {
                ed->wingBR_ins = ed->secWing_BR->GetAdjacentInserted(ed->pB, ed->wingPntBR);
            } else {
                if ( ed->wingOfWing_BR1 && ed->wingOfWing_BR1->treated) {
                    wingPnt_new_ins = ed->wingOfWing_BR1->GetAdjacentInserted(ed->wingPntBR, ed->pB);
                } else if ( ed->wingOfWing_BR2 && ed->wingOfWing_BR2->treated) {
                    wingPnt_new_ins = ed->wingOfWing_BR2->GetAdjacentInserted(ed->wingPntBR, ed->pB);
                } else {
                    wingPnt_new_ins = ed->wingPntBR;
                }

                ed->wingBR_ins = SplitEdge_AbsDist(krnl, obj, ed->pB, wingPnt_new_ins, ed->HN_weight[3],
                                                   ed->closeForWeld[3]);
                if ( !ed->wingBR_ins) {
                    return false;
                }
            }

            ed->outlBR_ins = ed->wingBR_ins;

            if ( treatedWing_BR) {
                if ( ed->wingOfWing_BR1 && ed->wingOfWing_BR1->treated) {
                    chk = findNextPointAlongNgn(krnl, obj, ed->wingBR_ins, ed->pB, ed->pB_ins, pn_wing_outl_far_ins);
                    if ( !chk) {
                        return false;
                    }
                } else {
                    pn_wing_outl_far_ins = ed->wing_BR->Get_AtoB_inserted(ed->pB, ed->pA);
                }

                ed->outlBR_ins = SplitEdge_AbsDist(krnl, obj, ed->pB_ins, pn_wing_outl_far_ins, ed->HN_weight[3]);
                if ( !ed->outlBR_ins) {
                    return false;
                }

                chk = ConnectPnts(krnl, obj, ed->wingBR_ins, ed->outlBR_ins, ed->pB_ins, ed->pB);
                if ( !chk) {
                    return false;
                }
            }
            // ------------------------------------------------------------------- connect (long slice) R
            if ( treatedWing_AR) {
                pn_plg_identif_1 = ed->pA_ins;
                pn_plg_identif_2 = ed->pB_ins;
            } else if ( treatedWing_AL) {
                pn_plg_identif_1 = ed->pA;
                pn_plg_identif_2 = ed->pA_ins;
            } else {
                pn_plg_identif_1 = ed->pA_ins;
                pn_plg_identif_2 = ed->pB_ins;
            }
            chk = ConnectPnts(krnl, obj, ed->outlBR_ins, ed->outlAR_ins, pn_plg_identif_1, pn_plg_identif_2);
            if ( !chk) {
                return false;
            }
        }
        ed->treated = true;
    }
    // //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // ////////////////////-------------------------------------------------------- pA_ins  &  pB_ins  Updating
    if ( CloseFans || ProlongOutlines || ( CompositeFirstPass && ConnectOutlines)) {
        for ( i = 0; i < ChargedEDaCount; i++) {
            EdgeDataStruct * ed = EdgeData[indEDa[i]];
            if ( ed->wing_AL) {
                ed->pA_ins = ed->wing_AL->GetAdjacentInserted(ed->pA, ed->pB);
            }
            if ( ed->wing_AR) {
                ed->pA_ins = ed->wing_AR->GetAdjacentInserted(ed->pA, ed->pB);
            }
            if ( ed->wing_BL) {
                ed->pB_ins = ed->wing_BL->GetAdjacentInserted(ed->pB, ed->pA);
            }
            if ( ed->wing_BR) {
                ed->pB_ins = ed->wing_BR->GetAdjacentInserted(ed->pB, ed->pA);
            }
        }
    }     // if CloseFans || ProlongOutlines || CompositeMode

    // ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // ------------------------------------------------------------------- Close Otlined Edges (fans)		/////////
    if ( CloseFans) {
        for ( i = 0; i < ChargedEDaCount; i++) {
            EdgeDataStruct * ed = EdgeData[indEDa[i]];
            EdgeDataStruct ** pFanEnd = &ed->fanEndAL;
            LONG * pFanCnt = &ed->fanCntAL;
            LONG * pWingIns = &ed->wingAL_ins;
            LONG * pp = &ed->pA;
            LONG * ppIns = &ed->pA_ins;
            LONG i, p1, p2, p3, pnew;
            Bool finded;
            // ------------------------------
            CHAR maxSides;
            if ( ed->open) {
                maxSides = 2;
            } else {
                maxSides = 4;
            }
            for ( s = 0; s < maxSides; s++) {
                if ( pFanCnt[s]) {
                    if ( ppIns[ s % 2] != pp[ s % 2]) {
                        p1 = ppIns[ s % 2];
                    } else {
                        p1 = ppIns[ (s % 2) ^ 1];
                    }
                    p2 = pWingIns[s];

                    Real weightWing = pFanEnd[s]->HN_weight[ ed->fanConterside[s]];
                    LONG StepsCnt = pFanCnt[s] - 1;

                    for ( i = 0; i < StepsCnt; i++) {
                        chk = GetNextPointInFan(krnl, obj, pp[ s % 2], p1, p2, p3, finded);
                        if ( !chk || !finded) {
                            return false;
                        }

                        Real rad = ( ed->HN_weight[s] * (Real)( StepsCnt - i) +
                                     weightWing * (Real)( i + 1)) / (Real)( StepsCnt + 1);
                        pnew = SplitEdge_AbsDist(krnl, obj, pp[ s % 2], p3, rad);

                        if ( !pnew) {
                            return false;
                        }
                        chk = ConnectPnts(krnl, obj, p2, pnew, pp[ s % 2], p2);
                        if ( !chk) {
                            return false;
                        }
                        p1 = p2;
                        p2 = pnew;
                    }

                    chk = GetNextPointInFan(krnl, obj, pp[ s % 2], p1, p2, p3, finded);
                    if ( !chk || !finded) {
                        return false;
                    }

                    chk = ConnectPnts(krnl, obj, p2, p3, pp[ s % 2], p2);
                    if ( !chk) {
                        return false;
                    }
                }
            }
        }
    }

    // /////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // ///////////-------------------------------/////////////////////////////////   prolong outlines
    for ( i = 0; i < ChargedEDaCount; i++) {
        if ( !ProlongOutlines) {
            break;
        }

        EdgeDataStruct * ed = EdgeData[indEDa[i]];

        if ( ed->open) {
            continue;
        }

        for ( s = 0; s < 4; s++) {
            if ( !ed->wing(s) && ed->wing(s ^ 2) && !( ed->connCheked[s ^ 2] & 2)) {
                chk = ConnectPnts(krnl, obj, ed->wing_ins(s), ed->p_ins(s), ed->p(s), ed->wing_ins(s));
                if ( !chk) {
                    return false;
                }
            }
        }
    }

    // /////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // ////////////////////////-------------------------------///////////////////   connect outlines
    Bool Need_ReInit_Modeling = ( ConnectOutlines && ConnOutlCnt) || Weld;

    if ( Need_ReInit_Modeling) {
        AMaPointMap_struct AMaPntMap;
        LONG * pntMap, mapCnt;

        krnl->Commit(obj, MODELING_COMMIT_CREATEMAP);           // // ! ! ! ! !

        chk = krnl->GetPointMap(obj, &pntMap, &mapCnt);
        if ( !chk) {
            return false;
        }

        LONG oldPntCnt = obj->GetPointCount();
        chk = AMaPntMap.AllocAMaPointMap(pntMap, mapCnt, oldPntCnt);
        if ( !chk) {
            return false;
        }

        Modeling::Free(krnl);
        krnl = Modeling::Alloc();
        if ( !krnl) {
            return false;
        }

        chk = krnl->InitObject(obj);
        if ( !chk) {
            return false;
        }

        // /////////////////////////////////////////////////////////////////
        if ( ConnectOutlines && ConnOutlCnt) {
            if ( CompositeFirstPass) {
                for ( connOutlChain = connOutlChainFirst; connOutlChain; connOutlChain = connOutlChain->next) {
                    connOutlChain->pnt1 = AMaPntMap.GetNum(connOutlChain->edgeStart->p_ins(connOutlChain->ab_Start));
                    connOutlChain->pnt2 = AMaPntMap.GetNum(connOutlChain->edgeEnd->p_ins(connOutlChain->ab_End));
                }
            }
            // //////////////////////////////////
            for ( i = 0; i < ConnOutlCnt; i++) {
                ConnectOutlinesDot_Struct * dot = arrDotsForConnOutl[i];
                EdgeDataStruct * edg2, * edg1;
                CHAR w1, w2;
                if ( dot->edge[0]->num > dot->edge[1]->num) {
                    edg2 = dot->edge[0];
                    edg1 = dot->edge[1];
                    w2 = dot->edgeSide[0];
                    w1 = dot->edgeSide[1];
                } else {
                    edg2 = dot->edge[1];
                    edg1 = dot->edge[0];
                    w2 = dot->edgeSide[1];
                    w1 = dot->edgeSide[0];
                }

                chk = krnl->MeltEdge(obj, NOTINDEX, AMaPntMap.GetNum(edg2->p_ins(w2)),
                                     AMaPntMap.GetNum(edg2->outl_ins(w2)));
                if ( !chk) {
                    return false;
                }

                chk = krnl->MeltEdge(obj, NOTINDEX, AMaPntMap.GetNum(edg2->wing_ins(w2)),
                                     AMaPntMap.GetNum(edg2->outl_ins(w2)));
                if ( !chk) {
                    return false;
                }

                chk = krnl->DeletePoint(obj, AMaPntMap.GetNum(edg2->outl_ins(w2)));
                if ( !chk) {
                    return false;
                }

                // ///
                if ( edg2->wing(w2 ^ 2)) {
                    if ( edg2->wing(w2 ^ 2)->num > edg2->num) {
                        CHAR wAnti = edg2->wing(w2 ^ 2)->GetCounterpart(edg2->p(w2), edg2->p(w2 ^ 1));
                        if ( !wAnti) {
                            return false;
                        } else {
                            wAnti ^= 4;
                        }
                        chk = krnl->MeltEdge(obj, NOTINDEX, AMaPntMap.GetNum(edg2->wing(w2 ^ 2)->wing_ins(wAnti)),
                                             AMaPntMap.GetNum(edg2->wing(w2 ^ 2)->outl_ins(wAnti)));
                    } else {
                        chk = krnl->MeltEdge(obj, NOTINDEX, AMaPntMap.GetNum(edg2->p_ins(w2 ^ 2)),
                                             AMaPntMap.GetNum(edg2->outl_ins(w2 ^ 2)));
                    }
                    if ( !chk) {
                        return false;
                    }
                }
                chk = krnl->DeletePoint(obj, AMaPntMap.GetNum(edg2->p_ins(w2)));
                if ( !chk) {
                    return false;
                }
                //
                if ( edg1->wing(w1 ^ 2)) {
                    if ( edg1->wing(w1 ^ 2)->num > edg1->num) {
                        CHAR wAnti = edg1->wing(w1 ^ 2)->GetCounterpart(edg1->p(w1), edg1->p(w1 ^ 1));
                        if ( !wAnti) {
                            return false;
                        } else {
                            wAnti ^= 4;
                        }
                        chk = krnl->MeltEdge(obj, NOTINDEX, AMaPntMap.GetNum(edg1->wing(w1 ^ 2)->wing_ins(wAnti)),
                                             AMaPntMap.GetNum(edg1->wing(w1 ^ 2)->outl_ins(wAnti)));
                    } else {
                        chk = krnl->MeltEdge(obj, NOTINDEX, AMaPntMap.GetNum(edg1->p_ins(w1 ^ 2)),
                                             AMaPntMap.GetNum(edg1->outl_ins(w1 ^ 2)));
                    }
                    if ( !chk) {
                        return false;
                    }
                }
                chk = krnl->DeletePoint(obj, AMaPntMap.GetNum(edg2->wing_ins(w2)));
                if ( !chk) {
                    return false;
                }

                // ///
                if ( edg2->secWing(w2 ^ 2) == edg1) {
                    chk = krnl->MeltEdge(obj, NOTINDEX, AMaPntMap.GetNum(edg2->p(w2)),
                                         AMaPntMap.GetNum(edg2->wing_ins(w2 ^ 2)));
                    if ( !chk) {
                        return false;
                    }
                    chk = krnl->DeletePoint(obj, AMaPntMap.GetNum(edg2->p(w2)));
                    if ( !chk) {
                        return false;
                    }
                } else if ( CompositeFirstPass) {
                    connOutlChain = dot->chain;
                    ConnectOutlinesChain_Struct * connOutlChainNew;
                    connOutlChainNew = (ConnectOutlinesChain_Struct *)GeAlloc(sizeof(ConnectOutlinesChain_Struct));
                    if ( !connOutlChainNew) {
                        return false;
                    }

                    connOutlChainNew->pnt1 = edg2->p(w2);
                    connOutlChainNew->pnt2 = connOutlChain->pnt2;
                    connOutlChain->pnt2 = edg2->p(w2);
                    connOutlChainNew->next = connOutlChain->next;
                    connOutlChain->next = connOutlChainNew;
                }
            }
        }
        // ///////////////////////////////////////////////////////////////////////////////// Weld
        if ( Weld) {
            for ( i = 0; i < ChargedEDaCount; i++) {
                EdgeDataStruct * ed = EdgeData[indEDa[i]];
                EdgeDataStruct ** pWing = &ed->wing_AL;
                EdgeDataStruct ** pWingOfWing = &ed->wingOfWing_AL1;
                LONG * pWingPnt = &ed->wingPntAL;
                LONG * pWingIns = &ed->wingAL_ins;
                for ( s = 0; s < 4; s++) {
                    if ( !pWing[s] && ed->closeForWeld[s] && !pWingOfWing[s * 2] && !pWingOfWing[s * 2 + 1]) {
                        chk = krnl->WeldPoints(obj, AMaPntMap.GetNum(pWingIns[s]), AMaPntMap.GetNum(pWingPnt[s]));
                        if ( !chk) {
                            return false;
                        }
                    }
                }
            }
        }
        // ///////////////////////////////////////////////////////////////////////       RE-SELECT connected edges
        if ( ConnectOutlines && ConnOutlCnt) {
            if ( CompositeFirstPass) {
                AMaPntMap.Free();
                nbr.Flush();
                //
                krnl->Commit(obj, MODELING_COMMIT_CREATEMAP);                   // // ! ! ! ! !
                chk = krnl->GetPointMap(obj, &pntMap, &mapCnt);
                if ( !chk) {
                    return false;
                }

                chk = AMaPntMap.AllocAMaPointMap(pntMap, mapCnt, oldPntCnt);
                if ( !chk) {
                    return false;
                }
                //
                Modeling::Free(krnl);
                krnlInited = false;
                //
                nbr.Init(obj->GetPointCount(), obj->GetPolygonR(), obj->GetPolygonCount(), NULL);

                LONG e, p1, p2;
                ConnectOutlinesChain_Struct * connOutlChain_next;
                connOutlChain = connOutlChainFirst;

                while ( connOutlChain) {
                    p1 = AMaPntMap.GetNum(connOutlChain->pnt1);
                    p2 = AMaPntMap.GetNum(connOutlChain->pnt2);
                    e = GetEdgeFromPoints(obj->GetPolygonR(), &nbr, p1, p2);
                    if ( e != -1) {
                        selE->Select(e);
                    }

                    connOutlChain_next = connOutlChain->next;
                    GeFree(connOutlChain);
                    connOutlChain = connOutlChain_next;
                }
            }
        }
        // ///////////////////////////////////////
        AMaPntMap.Free();
    }
    // /////////////////////////////////////////////////////////////////////////////////////////////////////////////

    if ( krnlInited) {
        krnl->Commit();
        Modeling::Free(krnl);
        krnlInited = false;
    }

    nbr.Flush();
    NbrInited = false;

    for ( i = 0; i < ConnOutlCnt; i++) {
        GeFree(arrDotsForConnOutl[i]);
    }
    GeFree(arrDotsForConnOutl);

    for ( i = 0; i < ChargedEDaCount; i++) {
        GeFree(EdgeData[indEDa[i]]);
    }
    GeFree(EdgeData);
    GeFree(indEDa);

    return true;
} // MakeChamf_In_AMa_PARALLEL_Mode

// ooooooooooooooooooooooooooooooo  functions lib  oooooooooooooooooooooooooooooooooooooooooooooooooooo
// WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW

// ----------IsEdgeDoubled----------------------------------------------------------------------
Bool IsEdgeDoubled(Neighbor * nbr, LONG e) {
    LONG E_In_P, P_of_E;
    PolyInfo * PlgInf;

    E_In_P = e % 4;
    P_of_E = ( e - E_In_P) / 4;
    PlgInf = nbr->GetPolyInfo(P_of_E);

    return ( PlgInf->mark[ E_In_P]);
}

// ----------Get_x4_edge_Points----------------------------------------------------------------------
void Get_x4_edge_points(const CPolygon * Plgons, LONG E_In_P, LONG P_of_E, LONG & pA, LONG & pB) {
    switch ( E_In_P) {
        case 0:
            pA = Plgons[ P_of_E].a;
            pB = Plgons[ P_of_E].b;
            break;
        case 1:
            pA = Plgons[ P_of_E].b;
            pB = Plgons[ P_of_E].c;
            break;
        case 2:
            pA = Plgons[ P_of_E].c;
            pB = Plgons[ P_of_E].d;
            break;
        case 3:
            pA = Plgons[ P_of_E].d;
            pB = Plgons[ P_of_E].a;
            break;
    }
}

void Get_x4_edge_points(const CPolygon * Plgons, LONG e, LONG & pA, LONG & pB) {
    LONG E_In_P = e % 4;
    LONG P_of_E = ( e - E_In_P) / 4;

    switch ( E_In_P) {
        case 0:
            pA = Plgons[ P_of_E].a;
            pB = Plgons[ P_of_E].b;
            break;
        case 1:
            pA = Plgons[ P_of_E].b;
            pB = Plgons[ P_of_E].c;
            break;
        case 2:
            pA = Plgons[ P_of_E].c;
            pB = Plgons[ P_of_E].d;
            break;
        case 3:
            pA = Plgons[ P_of_E].d;
            pB = Plgons[ P_of_E].a;
            break;
    }
}

// ----------Get_H_position_points----------------------------------------------------------------------
Bool Get_H_topol_points(Modeling * krnl, PolygonObject * obj, LONG pA, LONG pB, Bool & valid,
                        LONG & wingAL, LONG & wingAR, Bool & opened, LONG & wingBL, LONG & wingBR) {
    LONG i, j, edgNgnCnt = 0, a = 0, b = 0;

    valid = true;
    LONG * ngns = krnl->GetEdgeNgons(obj, pA, pB, edgNgnCnt);
    if ( !ngns) {
        return FALSE;
    }

    if ( !edgNgnCnt) {
        valid = false;
        return true;
    }
    opened = true;
    for ( i = 0; i < edgNgnCnt; i++) {
        Ngon Ngn;
        chk = krnl->GetNgon(obj, ngns[i], &Ngn);
        if ( !chk) {
            return FALSE;
        }

        for ( j = 0; j < Ngn.count; j++) {
            LONG hhhh = Ngn.points[j];              // ///////////////////
            if ( Ngn.points[j] == pB) {
                LONG j_pred = PredPointIndInNgn(&Ngn, j);
                if ( Ngn.points[j_pred] == pA) {
                    a = Ngn.points[ PredPointIndInNgn(&Ngn, j_pred)];
                    b = Ngn.points[ NextPointIndInNgn(&Ngn, j)];
                } else {
                    LONG j_next = NextPointIndInNgn(&Ngn, j);
                    if ( Ngn.points[j_next] == pA) {
                        a = Ngn.points[ NextPointIndInNgn(&Ngn, j_next)];
                        b = Ngn.points[ PredPointIndInNgn(&Ngn, j)];
                    }
                }
                break;
            }
        }
        Ngn.Free();

        if ( i == 0) {
            wingAL = a;
            wingBL = b;
        } else {
            opened = false;
            wingAR = a;
            wingBR = b;
        }
    }
    krnl->FreeTable(obj, ngns);
    return true;
} // Get_H_topol_points

// -----------PredPointIndInNgn--------------------------------------------------------------------
LONG PredPointIndInNgn(Ngon * Ngn, LONG p) {
    if ( Ngn->segcount == 1) {
        if ( p == 0) {
            return ( Ngn->count - 1);
        } else {
            return ( p - 1);
        }
    } else {
        LONG i, segStart = 0, segEnd = 0, summ = 0;
        for ( i = 0; i < Ngn->segcount; i++) {
            summ += Ngn->segments[i];
            if ( summ > p) {
                segEnd = summ;
                break;
            }
            segStart = summ;
        }
        if ( p == segStart) {
            return ( segEnd - 1);
        } else {
            return ( p - 1);
        }
    }
}

// -----------NextPointIndInNgn--------------------------------------------------------------------
LONG NextPointIndInNgn(Ngon * Ngn, LONG p) {
    if ( Ngn->segcount == 1) {
        if ( p == ( Ngn->count - 1)) {
            return 0;
        } else {
            return ( p + 1);
        }
    } else {
        LONG i, segStart = 0, segEnd = 0, summ = 0;
        for ( i = 0; i < Ngn->segcount; i++) {
            summ += Ngn->segments[i];
            if ( summ > p) {
                segEnd = summ;
                break;
            }
            segStart = summ;
        }
        if ( p == ( segEnd - 1)) {
            return segStart;
        } else {
            return ( p + 1);
        }
    }
}

// -----------findPredPointInNgn--------------------------------------------------------------------
Bool findPredPointInNgn(Ngon * Ngn, LONG p, LONG & prP) {
    LONG i;
    Bool found = false;

    for ( i = 0; i < Ngn->count; i++) {
        if ( Ngn->points[i] == p) {
            found = true;
            break;
        }
    }
    if ( found) {
        prP = Ngn->points[ PredPointIndInNgn(Ngn, i)];
        return true;
    } else {
        return false;
    }
}

// -----------findNextPointInNgn--------------------------------------------------------------------
Bool findNextPointInNgn(Ngon * Ngn, LONG p, LONG & prP) {
    LONG i;
    Bool found = false;

    for ( i = 0; i < Ngn->count; i++) {
        if ( Ngn->points[i] == p) {
            found = true;
            break;
        }
    }
    if ( found) {
        prP = Ngn->points[ NextPointIndInNgn(Ngn, i)];
        return true;
    } else {
        return false;
    }
}

// -----------findNextPointAlongNgn--------------------------------------------------------------------
Bool findNextPointAlongNgn(Modeling * krnl, PolygonObject * obj, LONG p1, LONG p2, LONG p3, LONG & p4) {
    LONG ngn;
    Ngon Ngn;

    chk = GetNgnFrom3pnts(krnl, obj, p1, p2, p3, ngn);
    if ( !chk) {
        return false;
    }

    chk = krnl->GetNgon(obj, ngn, &Ngn);
    if ( !chk) {
        return false;
    }

    chk = findNextPointInNgn(&Ngn, p3, p4);
    if ( !chk) {
        return false;
    }

    if ( p4 == p2) {
        chk = findPredPointInNgn(&Ngn, p3, p4);
        if ( !chk) {
            return false;
        }
    }
    return true;
}

// ----------Compare_pBaseObjects----------------------------------------------------------------------
/*Bool Compare_pBaseObjects( BaseObject *Ob, BaseObject *cacheOb)
   {
        while( cacheOb)
        {
                if( Ob == cacheOb)
                        return TRUE ;
                else
                        cacheOb = cacheOb->GetCacheParent() ;
        }
        return FALSE ;
   }*/

// ----------SelectOpenEdges-------------------------------------------------------------------------
void SelectOpenEdges(PolygonObject * obj, Neighbor * nbr, BaseSelect * selE) {
    LONG plgCnt = obj->GetPolygonCount();
    LONG edgCnt = plgCnt * 4;
    const CPolygon * ObjPlgns = obj->GetPolygonR();
    LONG i, p1, p2;

    for ( i = 0; i < plgCnt; i++) {
        nbr->GetEdgePolys(ObjPlgns[i].a, ObjPlgns[i].b, &p1, &p2);
        if ((p1 < 0) || (p2 < 0)) {
            selE->Select(i * 4);
        }
        nbr->GetEdgePolys(ObjPlgns[i].b, ObjPlgns[i].c, &p1, &p2);
        if ((p1 < 0) || (p2 < 0)) {
            selE->Select(i * 4 + 1);
        }
        if ( ObjPlgns[i].c != ObjPlgns[i].d) {
            nbr->GetEdgePolys(ObjPlgns[i].c, ObjPlgns[i].d, &p1, &p2);
            if ((p1 < 0) || (p2 < 0)) {
                selE->Select(i * 4 + 2);
            }
        }
        nbr->GetEdgePolys(ObjPlgns[i].d, ObjPlgns[i].a, &p1, &p2);
        if ((p1 < 0) || (p2 < 0)) {
            selE->Select(i * 4 + 3);
        }
    }
}

// ----------SelectEdgesWithThrAngle------------------------------------------------------------------
void SelectEdgesWithThrAngle(PolygonObject * obj, Neighbor * nbr, Modeling * krnl, BaseSelect * selE, Real ang, Bool andOpen) {
    Real thresh = Cos(Rad(ang));
    LONG plgCnt = obj->GetPolygonCount();
    const CPolygon * ObjPlgns = obj->GetPolygonR();
    PolyInfo * PlgInf;
    Vector normalA, normalB;
    LONG i, side, a = 0, b = 0, edgNgnCnt;

    for ( i = 0; i < plgCnt; i++) {
        PlgInf = nbr->GetPolyInfo(i);
        for ( side = 0; side < 4; side++) {
            if ( PlgInf->mark[side]) {
                continue;
            }
            switch ( side) {
                case 0: a = ObjPlgns[i].a; b = ObjPlgns[i].b; break;
                case 1: a = ObjPlgns[i].b; b = ObjPlgns[i].c; break;
                case 2: a = ObjPlgns[i].c; b = ObjPlgns[i].d; break;
                case 3: a = ObjPlgns[i].d; b = ObjPlgns[i].a; break;
            }
            LONG * ngns = krnl->GetEdgeNgons(obj, a, b, edgNgnCnt);
            if ((edgNgnCnt == 1) && andOpen) {
                krnl->FreeTable(obj, ngns);
                selE->Select(i * 4 + side);
                continue;
            }
            if ( edgNgnCnt != 2) {
                krnl->FreeTable(obj, ngns);
                continue;
            }
            krnl->GetNgonNormal(obj, ngns[0], &normalA);
            krnl->GetNgonNormal(obj, ngns[1], &normalB);
            if ( normalA * normalB < thresh) {
                selE->Select(i * 4 + side);
            }
            krnl->FreeTable(obj, ngns);
        }
    }
}

// ----------GetEdgeFromPoints----------------------------------------------------------------------
LONG GetEdgeFromPoints(const CPolygon * plgns, Neighbor * nbr, LONG p1, LONG p2) {
    LONG j, iip, plg1, plg2, pnt_in_plg_1 = 0, pnt_in_plg_2 = 0, edg_in_plg, plg0;
    PolyInfo * PlgInf;

    nbr->GetEdgePolys(p1, p2, &plg1, &plg2);
    plg0 = LMin(plg1, plg2);
    if ( plg0 < 0) {
        plg0 = LMax(plg1, plg2);
    }

    for ( iip = 0; iip < 2; iip++) {
        for ( j = 0; j < 4; j++) {
            if ( p1 == Point_From_Plg(&(plgns[plg0]), j)) {
                pnt_in_plg_1 = j;
                break;
            }
        }
        for ( j = 0; j < 4; j++) {
            if ( p2 == Point_From_Plg(&(plgns[plg0]), j)) {
                pnt_in_plg_2 = j;
                break;
            }
        }
        if ( !( pnt_in_plg_1 || pnt_in_plg_2)) {
            return -1;
        }
        if ( Abs(pnt_in_plg_1 - pnt_in_plg_2) > 1) {
            edg_in_plg = 3;
        } else {
            edg_in_plg = LMin(pnt_in_plg_1, pnt_in_plg_2);
        }

        PlgInf = nbr->GetPolyInfo(plg0);
        if ( !PlgInf->mark[edg_in_plg]) {
            return ( plg0 * 4 + edg_in_plg);
        } else {
            plg0 = LMax(plg1, plg2);
        }
    }
    return -1;
}

// ----------Point_From_Plg-------------------------------------------------------------------------
LONG Point_From_Plg(const CPolygon * Plg, LONG i) {
    switch ( i) {
        case 0: return Plg->a;
        case 1: return Plg->b;
        case 2: return Plg->c;
        case 3: return Plg->d;
    }
    return -1;
}

// ----------GetNgnFrom3pnts-------------------------------------------------------------------------
Bool GetNgnFrom3pnts(Modeling * krnl, PolygonObject * obj, LONG p1, LONG p2, LONG p3, LONG & ngn) {
    LONG i, j, edgNgnCnt_12, edgNgnCnt_23;
    LONG * ngns_12 = krnl->GetEdgeNgons(obj, p1, p2, edgNgnCnt_12);
    LONG * ngns_23 = krnl->GetEdgeNgons(obj, p2, p3, edgNgnCnt_23);

    for ( i = 0; i < edgNgnCnt_12; i++) {
        for ( j = 0; j < edgNgnCnt_23; j++) {
            if ( ngns_12[i] == ngns_23[j]) {
                ngn = ngns_12[i];
                krnl->FreeTable(obj, ngns_12);
                krnl->FreeTable(obj, ngns_23);
                return true;
            }
        }
    }
    return false;
}

// ----------GetNextPointInFan----------------------------------------------------------------------
Bool GetNextPointInFan(Modeling * krnl, PolygonObject * obj, LONG pc, LONG p1, LONG p2, LONG & p3, Bool & found) {
    LONG i, j, edgNgnCnt_c1, edgNgnCnt_c2, nextNgn = 0;
    LONG * ngns_c1 = krnl->GetEdgeNgons(obj, pc, p1, edgNgnCnt_c1);
    LONG * ngns_c2 = krnl->GetEdgeNgons(obj, pc, p2, edgNgnCnt_c2);
    Ngon Ngn;

    found = false;

    for ( i = 0; i < edgNgnCnt_c2; i++) {
        if ( !edgNgnCnt_c1) {
            break;
        }
        Bool notInC1 = true;
        for ( j = 0; j < edgNgnCnt_c1; j++) {
            if ( ngns_c2[i] == ngns_c1[j]) {
                notInC1 = false;
            }
        }
        if ( notInC1) {
            nextNgn = ngns_c2[i];
            found = true;
            break;
        }
    }
    if ( !found) {
        goto Exit;
    }

    chk = krnl->GetNgon(obj, nextNgn, &Ngn);
    if ( !chk) {
        return false;
    }

    LONG pTmp;
    chk = findPredPointInNgn(&Ngn, pc, pTmp);
    if ( !chk) {
        return false;
    }

    if ( pTmp == p2) {
        chk = findNextPointInNgn(&Ngn, pc, p3);
        if ( !chk) {
            return false;
        }
    } else {
        p3 = pTmp;
    }

    Ngn.Free();
    Exit:
    krnl->FreeTable(obj, ngns_c1);
    krnl->FreeTable(obj, ngns_c2);

    return true;
} // GetNextPointInFan

// ----------SplitEdge_AbsDist----------------------------------------------------------------------
LONG SplitEdge_AbsDist(Modeling * krnl, PolygonObject * obj, LONG p1, LONG p2, Real ChamfRadius, Bool & tooClose) {
    Vector vec1, vec2;
    Real relat_pos_in_edge, len;

    chk = krnl->GetPoint(obj, p1, &vec1);
    if ( !chk) {
        return 0;
    }

    chk = krnl->GetPoint(obj, p2, &vec2);
    if ( !chk) {
        return 0;
    }

    len = Len(vec2 - vec1);
    relat_pos_in_edge = ChamfRadius / len;
    if ( relat_pos_in_edge >= 1) {
        tooClose = true;
    }

    return ( krnl->SplitEdge(obj, p1, p2, relat_pos_in_edge) );
}

LONG SplitEdge_AbsDist(Modeling * krnl, PolygonObject * obj, LONG p1, LONG p2, Real ChamfRadius) {
    Vector vec1, vec2;
    Real relat_pos_in_edge, len;

    chk = krnl->GetPoint(obj, p1, &vec1);
    if ( !chk) {
        return 0;
    }

    chk = krnl->GetPoint(obj, p2, &vec2);
    if ( !chk) {
        return 0;
    }

    len = Len(vec2 - vec1);
    relat_pos_in_edge = ChamfRadius / len;

    return ( krnl->SplitEdge(obj, p1, p2, relat_pos_in_edge) );
}

// ----------ConnectPnts----------------------------------------------------------------------
Bool ConnectPnts(Modeling * krnl, PolygonObject * obj, LONG p1, LONG p2pl1, LONG pl2, LONG pl3) {
    LONG ngn;

    chk = GetNgnFrom3pnts(krnl, obj, p2pl1, pl2, pl3, ngn);
    if ( !chk) {
        return false;
    }
    LONG newNgn = krnl->SplitPolygon(obj, ngn, p1, p2pl1);
    if ( !newNgn) {
        return false;
    }
    return true;
}
