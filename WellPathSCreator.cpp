#include <slvs.h>

#include <vector>
#include <assert.h>
#include <valarray>
#include <tuple>
#include <iostream>

class SolveSpaceSystem
{
public:
    SolveSpaceSystem() 
    {
        m_paramsMemory.reserve(100);
        m_entityMemory.reserve(100);
        m_constraintMemory.reserve(100);
    }

    Slvs_hParam addParam(Slvs_Param parameter)  
    { 
        parameter.h = static_cast<Slvs_hParam>(m_paramsMemory.size()+1);
        m_paramsMemory.push_back(parameter);

        m_slvsSystem.param  =  m_paramsMemory.data();
        m_slvsSystem.params =  static_cast<int>(m_paramsMemory.size());

        return parameter.h;
    }

    Slvs_hEntity addEntity(Slvs_Entity entity)  
    {
        entity.h = static_cast<Slvs_hEntity>(m_entityMemory.size()+1);
        m_entityMemory.push_back(entity);

        m_slvsSystem.entity   =  m_entityMemory.data();
        m_slvsSystem.entities = static_cast<int>(m_entityMemory.size());

        return entity.h; 
    }

    Slvs_hConstraint addConstr(Slvs_Constraint constr)   
    {
        constr.h = static_cast<Slvs_hConstraint>(m_constraintMemory.size()+1);
        m_constraintMemory.push_back(constr);

        m_slvsSystem.constraint  =  m_constraintMemory.data();
        m_slvsSystem.constraints = static_cast<int>(m_constraintMemory.size());

        return constr.h; 
    }

    enum ResultStatus {
        RESULT_OKAY              = SLVS_RESULT_OKAY             ,
        RESULT_INCONSISTENT      = SLVS_RESULT_INCONSISTENT     ,
        RESULT_DIDNT_CONVERGE    = SLVS_RESULT_DIDNT_CONVERGE   ,
        RESULT_TOO_MANY_UNKNOWNS = SLVS_RESULT_TOO_MANY_UNKNOWNS,
    };

    ResultStatus solve(Slvs_hGroup groupId, bool reportFailedConstraints = true)
    {
        m_failedConstrMemory.resize(m_constraintMemory.size());

        m_slvsSystem.failed  =  m_failedConstrMemory.data();
        m_slvsSystem.faileds = static_cast<int>(m_failedConstrMemory.size());

        m_slvsSystem.calculateFaileds = reportFailedConstraints;

        Slvs_Solve(&m_slvsSystem, groupId);

        m_failedConstrMemory.resize(m_slvsSystem.faileds);

        return static_cast<ResultStatus>(m_slvsSystem.result);
    }

    double parameterValue(Slvs_hParam paramId)
    {
        return m_paramsMemory[paramId-1].val;
    }

    std::tuple< std::valarray<double>,
        std::valarray<double>,
        std::valarray<double> >
        orientationMx(Slvs_hEntity normalIn3dEntityId)
    {
        Slvs_Entity e_CS = m_entityMemory[normalIn3dEntityId -1];
        if ( e_CS.type == SLVS_E_NORMAL_IN_3D )
        {
            std::valarray<double> quat ={ 0.0, 0.0, 0.0, 0.0 };
            quat[0] =  parameterValue(e_CS.param[0]);
            quat[1] =  parameterValue(e_CS.param[1]);
            quat[2] =  parameterValue(e_CS.param[2]);
            quat[3] =  parameterValue(e_CS.param[3]);
            std::valarray<double> Ex ={ 0.0,0.0,0.0 };
            std::valarray<double> Ey ={ 0.0,0.0,0.0 };
            std::valarray<double> Ez ={ 0.0,0.0,0.0 };

            Slvs_QuaternionU(quat[0], quat[1], quat[2], quat[3],
                             &Ex[0], &Ex[1], &Ex[2]);
            Slvs_QuaternionV(quat[0], quat[1], quat[2], quat[3],
                             &Ey[0], &Ey[1], &Ey[2]);
            Slvs_QuaternionN(quat[0], quat[1], quat[2], quat[3],
                             &Ez[0], &Ez[1], &Ez[2]);

            return std::make_tuple(Ex, Ey, Ez);
        }
        assert(false);
        return std::make_tuple(std::valarray<double>(), std::valarray<double>(), std::valarray<double>());
    }


    // Returns point as x, y, z values
    std::valarray<double> global3DPos (Slvs_hEntity pointEntityId)
    {
        std::valarray<double> point = {0.0,0.0,0.0};

        Slvs_Entity pointEntity = m_entityMemory[pointEntityId -1];
        if (pointEntity.type == SLVS_E_POINT_IN_2D)
        {
            std::valarray<double> locPoint = {0.0,0.0,0.0};
            locPoint[0] = parameterValue(pointEntity.param[0]);
            locPoint[1] = parameterValue(pointEntity.param[1]);

            Slvs_Entity e_Plane = m_entityMemory[pointEntity.wrkpl - 1];
            std::valarray<double> origin = global3DPos(e_Plane.point[0]);
            auto mx = orientationMx(e_Plane.normal);
            point = origin + std::get<0>(mx)*locPoint[0] + std::get<1>(mx)*locPoint[1];

        }
        else if (pointEntity.type == SLVS_E_POINT_IN_3D)
        {
            point[0] = parameterValue(pointEntity.param[0]);
            point[1] = parameterValue(pointEntity.param[1]);
            point[2] = parameterValue(pointEntity.param[2]);
        }

        return point;
    }

    Slvs_Constraint constraint(Slvs_hConstraint constraintId) { return m_constraintMemory[constraintId-1]; }
    const std::vector<Slvs_hConstraint>& failedConstraints() { return m_failedConstrMemory;}

private:
    Slvs_System m_slvsSystem;

    std::vector<Slvs_Param> m_paramsMemory;
    std::vector<Slvs_Entity> m_entityMemory;
    std::vector<Slvs_Constraint> m_constraintMemory;
    std::vector<Slvs_hConstraint> m_failedConstrMemory;
};
 
/*-----------------------------------------------------------------------------
 Unfinished. Only reference comments and code.
 Some problem with constraining the normal of a plane. Seems like we constrain 
 the complete orientation, and not only the normal vector.
*---------------------------------------------------------------------------*/
extern "C" void wellPathTest(double p1x, 
                  double p1y,
                  double p1z, 
                  double azi1, 
                  double inc1, 
                  double p2x, 
                  double p2y,
                  double p2z, 
                  double azi2, 
                  double inc2)
{
    /*
    Group 1
    P1 (P1x, P1y, P1z)
    Plane1 ( P1, Quat Identity)

    P2 (P2x, P2y, P2z)
    Plane4 ( P2, Quat Identity)

    Group 2

    // From top 

    // Azimuth

    Plane2 (P1,  X-Axis)
    Perpendicular( Plane2. Normal, Z-Axis)
    Angle (Plane2.Normal, X-Axis, Azimuth1 )

    L2P2( l2p2x = 10, l2p2y = 0)  
    Line2 (Plane2, P1, L2P2)
    Horizontal(Plane2, Line2)
    Coincident(Plane1, L2P2)

    // Dip line

    L3P2(l3p2x = 0.0, l3p2y = -10)
    Line3 (Plane2, P1, L3P2)
    Angle(Line2, Line3, Dip1 )

    // Arc plane

    Plane3 (L3P2, X-Axis)
    Coincident (Plane3, P1)

    // Arc 1

    A1P2(a1p2x = 10, a1p2y = -10)
    A1C(a1cx = 10, a1cy = 0)
    Arc1 (Plane3, P1, A1P2, A1C)
    Tangent (Arc1, Line3)

    // Arc1 to Arc 2 line

    L4P2(l4p2x = ??, l4p2y = ??)
    Line4(Plane3, A1P2, L4P2)
    Tangent(Arc1, Line4)

    // From Bottom

    // Azimuth

    Plane5(P2, X-Axis)
    Perpendicular(Plane5.Normal, Z-Axis)
    Angle (Plane5.Normal, X-axis, Azimuth2 )

    L6P2( l6p2x = 10, l6p2y = 0)  
    Line6 (Plane5, P2, L6P2)
    Horizontal(Plane5, Line6)
    Coincident(Plane4, L6P2)

    // Dip line

    L7P2(l7p2x = 0.0, l7p2y = -10)
    Line7 (Plane5, P2, L7P2)
    Angle(Line6, Line7, Dip2 )

    // Arc plane

    Plane6 (L7P2, X-Axis)
    Coincident (Plane6, P2)

    // Arc 2

    A2P2(a2p2x = 10, a2p2y = -10)
    A2C(a2cx = 10, a2cy = 0)
    Arc2 (Plane6, P2, A2P2, A2C)
    Tangent (Arc1, Line3)

    // Connection the top and bottom

    Coincident(Plane6, L4P2)
    Coincident(Plane6, A1P2)
    Coincident(A2P2, L4P2)
    Tangent(Arc2, Line4)

    // The dogleg constraints

    Diameter(Arc1, 2*Radius1)
    Diamater(Arc2, 2*Radius2)

    */

    SolveSpaceSystem sys;

    Slvs_hGroup group1 = 1;
    Slvs_hGroup group2 = 2;


    Slvs_hParam p_p1x  = sys.addParam( Slvs_MakeParam(-1, group1, p1x) ); 
    Slvs_hParam p_p1y  = sys.addParam( Slvs_MakeParam(-1, group1, p1y) ); 
    Slvs_hParam p_p1z  = sys.addParam( Slvs_MakeParam(-1, group1, p1z) );  
    Slvs_hParam p_azi1 = sys.addParam( Slvs_MakeParam(-1, group1, azi1) ); 
    Slvs_hParam p_inc1 = sys.addParam( Slvs_MakeParam(-1, group1, inc1) ); 

    Slvs_hParam p_p2x  = sys.addParam( Slvs_MakeParam(-1, group1, p2x) ); 
    Slvs_hParam p_p2y  = sys.addParam( Slvs_MakeParam(-1, group1, p2y) ); 
    Slvs_hParam p_p2z  = sys.addParam( Slvs_MakeParam(-1, group1, p2z) );  
    Slvs_hParam p_azi2 = sys.addParam( Slvs_MakeParam(-1, group1, azi2) ); 
    Slvs_hParam p_inc2 = sys.addParam( Slvs_MakeParam(-1, group1, inc2) ); 

    double unitQw, unitQx, unitQy, unitQz;
    Slvs_MakeQuaternion(1, 0, 0,
                        0, 1, 0, 
                        &unitQw, &unitQx, &unitQy, &unitQz);

    Slvs_hParam p_unitQw = sys.addParam( Slvs_MakeParam(-1, group1, unitQw) );  
    Slvs_hParam p_unitQx = sys.addParam( Slvs_MakeParam(-1, group1, unitQx) ); 
    Slvs_hParam p_unitQy = sys.addParam( Slvs_MakeParam(-1, group1, unitQy) ); 
    Slvs_hParam p_unitQz = sys.addParam( Slvs_MakeParam(-1, group1, unitQz) );
    Slvs_hEntity e_zAxisNorm = sys.addEntity( Slvs_MakeNormal3d(-1, group1, p_unitQw, p_unitQx,  p_unitQy,  p_unitQz) );

    double xAxisQw, xAxisQx, xAxisQy, xAxisQz;
    Slvs_MakeQuaternion(0, 1, 0,
                        0, 0, 1, 
                        &xAxisQw, &xAxisQx, &xAxisQy, &xAxisQz);

    Slvs_hParam p_xAxisQw = sys.addParam( Slvs_MakeParam(-1, group1, xAxisQw) );  
    Slvs_hParam p_xAxisQx = sys.addParam( Slvs_MakeParam(-1, group1, xAxisQx) ); 
    Slvs_hParam p_xAxisQy = sys.addParam( Slvs_MakeParam(-1, group1, xAxisQy) ); 
    Slvs_hParam p_xAxisQz = sys.addParam( Slvs_MakeParam(-1, group1, xAxisQz) );
    Slvs_hEntity e_xAxisNorm = sys.addEntity( Slvs_MakeNormal3d(-1, 
                                                                group1, 
                                                                p_xAxisQw, 
                                                                p_xAxisQx, 
                                                                p_xAxisQy, 
                                                                p_xAxisQz));



    Slvs_hEntity e_P1 = sys.addEntity( Slvs_MakePoint3d(-1, group1, p_p1x, p_p1y, p_p1z) );
    Slvs_hEntity e_Plane1 = sys.addEntity( Slvs_MakeWorkplane(-1, group1, e_P1, e_zAxisNorm) );

    Slvs_hEntity e_P2 = sys.addEntity( Slvs_MakePoint3d(-1, group1, p_p2x, p_p2y, p_p2z) );
    Slvs_hEntity e_Plane4 = sys.addEntity( Slvs_MakeWorkplane(-1, group1, e_P2, e_zAxisNorm) );



    // Azimuth

    Slvs_hParam p_Plane2NormQw = sys.addParam( Slvs_MakeParam(-1, group2, xAxisQw) );  
    Slvs_hParam p_Plane2NormQx = sys.addParam( Slvs_MakeParam(-1, group2, xAxisQx) ); 
    Slvs_hParam p_Plane2NormQy = sys.addParam( Slvs_MakeParam(-1, group2, xAxisQy) ); 
    Slvs_hParam p_Plane2NormQz = sys.addParam( Slvs_MakeParam(-1, group2, xAxisQz));
    Slvs_hEntity e_Plane2Norm = sys.addEntity( Slvs_MakeNormal3d(-1, group2,
                                                                 p_Plane2NormQw,
                                                                 p_Plane2NormQx,
                                                                 p_Plane2NormQy,
                                                                 p_Plane2NormQz));


    Slvs_hEntity e_Plane2 = sys.addEntity( Slvs_MakeWorkplane(-1, group2, e_P1, e_Plane2Norm) );


    Slvs_hConstraint c_Plane2Perp = sys.addConstr(Slvs_MakeConstraint(-1, 
                                                                      group2,
                                                                      SLVS_C_PERPENDICULAR,
                                                                      SLVS_FREE_IN_3D,
                                                                      0.0,
                                                                      0,
                                                                      0,
                                                                      e_Plane2Norm,
                                                                      e_zAxisNorm 
                                                                      ));

    Slvs_hConstraint c_Plane2Azi1 = sys.addConstr(Slvs_MakeConstraint(-1, 
                                                                      group2,
                                                                      SLVS_C_ANGLE,
                                                                      SLVS_FREE_IN_3D,//e_Plane2,
                                                                      azi1,
                                                                      0,
                                                                      0,
                                                                      e_Plane2Norm, 
                                                                      e_xAxisNorm ));
    auto solveResult = sys.solve(group2, true);
    assert(solveResult == SolveSpaceSystem::RESULT_OKAY);


    Slvs_hParam p_L2P2x  = sys.addParam( Slvs_MakeParam(-1, group2, 10) );  
    Slvs_hParam p_L2P2y  = sys.addParam( Slvs_MakeParam(-1, group2, 0) ); 
    Slvs_hEntity e_L2P2  = sys.addEntity( Slvs_MakePoint2d(-1, group2, e_Plane2, p_L2P2x, p_L2P2y) );
    Slvs_hEntity e_Line2 = sys.addEntity( Slvs_MakeLineSegment(-1, group2, e_Plane2, e_P1, e_L2P2) );
    Slvs_hConstraint c_L2Hor = sys.addConstr(Slvs_MakeConstraint(-1, 
                                                                 group2,
                                                                 SLVS_C_HORIZONTAL,
                                                                 e_Plane2,
                                                                 0.0,
                                                                 0,
                                                                 0,
                                                                 e_Line2,
                                                                 0));
    Slvs_hConstraint c_L2Plane1 = sys.addConstr(Slvs_MakeConstraint(-1,
                                                                    group2,
                                                                    SLVS_C_PT_IN_PLANE,
                                                                    SLVS_FREE_IN_3D,
                                                                    0.0,
                                                                    e_L2P2,
                                                                    0,
                                                                    e_Plane1,
                                                                    0));


}

/*-----------------------------------------------------------------------------
*
*---------------------------------------------------------------------------*/
extern "C" void wellPathTest2(double p1x, 
                             double p1y,
                             double p1z, 
                             double azi1, 
                             double inc1,
                             double rad1, 
                             double p2x, 
                             double p2y,
                             double p2z, 
                             double azi2, 
                             double inc2, 
                             double rad2)
{
    SolveSpaceSystem sys;

    Slvs_hGroup group1 = 1;
    Slvs_hGroup group2 = 2;

    // Group 1, Fixed
    // P1
    Slvs_hParam p_p1x  = sys.addParam( Slvs_MakeParam(-1, group1, p1x) ); 
    Slvs_hParam p_p1y  = sys.addParam( Slvs_MakeParam(-1, group1, p1y) ); 
    Slvs_hParam p_p1z  = sys.addParam( Slvs_MakeParam(-1, group1, p1z) );  

    Slvs_hEntity e_P1 = sys.addEntity( Slvs_MakePoint3d(-1, group1, p_p1x, p_p1y, p_p1z) );

    // PT1
    double pt1x = p1x + sin(azi1)*sin(inc1);
    double pt1y = p1y + cos(azi1)*sin(inc1); 
    double pt1z = p1z - cos(inc1);

    Slvs_hParam p_pt1x  = sys.addParam( Slvs_MakeParam(-1, group1, pt1x) ); 
    Slvs_hParam p_pt1y  = sys.addParam( Slvs_MakeParam(-1, group1, pt1y) ); 
    Slvs_hParam p_pt1z  = sys.addParam( Slvs_MakeParam(-1, group1, pt1z) );  

    Slvs_hEntity e_PT1 = sys.addEntity( Slvs_MakePoint3d(-1, group1, p_pt1x, p_pt1y, p_pt1z) );

    // Tangent Line 1

    Slvs_hEntity e_LT1 = sys.addEntity(Slvs_MakeLineSegment(-1, group1, SLVS_FREE_IN_3D, e_P1, e_PT1));

    // P2
    Slvs_hParam p_p2x  = sys.addParam( Slvs_MakeParam(-1, group1, p2x) ); 
    Slvs_hParam p_p2y  = sys.addParam( Slvs_MakeParam(-1, group1, p2y) ); 
    Slvs_hParam p_p2z  = sys.addParam( Slvs_MakeParam(-1, group1, p2z) );  

    Slvs_hEntity e_P2 = sys.addEntity( Slvs_MakePoint3d(-1, group1, p_p2x, p_p2y, p_p2z) );

    // PT2
    double pt2x = p2x + sin(azi2)*sin(inc2);
    double pt2y = p2y + cos(azi2)*sin(inc2); 
    double pt2z = p2z - cos(inc2);

    Slvs_hParam p_pt2x  = sys.addParam( Slvs_MakeParam(-1, group1, pt2x) ); 
    Slvs_hParam p_pt2y  = sys.addParam( Slvs_MakeParam(-1, group1, pt2y) ); 
    Slvs_hParam p_pt2z  = sys.addParam( Slvs_MakeParam(-1, group1, pt2z) );  

    Slvs_hEntity e_PT2 = sys.addEntity( Slvs_MakePoint3d(-1, group1, p_pt2x, p_pt2y, p_pt2z) );

    // Tangent Line 2

    Slvs_hEntity e_LT2 = sys.addEntity(Slvs_MakeLineSegment(-1, group1, SLVS_FREE_IN_3D, e_P2, e_PT2));

    // Plane1
 
    double unitQw, unitQx, unitQy, unitQz;
    Slvs_MakeQuaternion(1, 0, 0,
                        0, 1, 0, 
                        &unitQw, &unitQx, &unitQy, &unitQz);

    // Plane 1

    Slvs_hParam p_Plane1Qw = sys.addParam( Slvs_MakeParam(-1, group2, unitQw) );  
    Slvs_hParam p_Plane1Qx = sys.addParam( Slvs_MakeParam(-1, group2, unitQx) ); 
    Slvs_hParam p_Plane1Qy = sys.addParam( Slvs_MakeParam(-1, group2, unitQy) ); 
    Slvs_hParam p_Plane1Qz = sys.addParam( Slvs_MakeParam(-1, group2, unitQz));
    Slvs_hEntity e_Plane1Q = sys.addEntity( Slvs_MakeNormal3d(-1, group2,
                                                                 p_Plane1Qw,
                                                                 p_Plane1Qx,
                                                                 p_Plane1Qy,
                                                                 p_Plane1Qz));
    Slvs_hEntity e_Plane1 = sys.addEntity( Slvs_MakeWorkplane(-1, group2, e_P1, e_Plane1Q) );

    Slvs_hConstraint c_PT1Plane1 = sys.addConstr(Slvs_MakeConstraint(-1,
                                                                    group2,
                                                                    SLVS_C_PT_IN_PLANE,
                                                                    SLVS_FREE_IN_3D,
                                                                    0.0,
                                                                    e_PT1,
                                                                    -1,
                                                                    e_Plane1,
                                                                    -1));
    // Arc1 center
    Slvs_hParam p_c1x  = sys.addParam( Slvs_MakeParam(-1, group2, 10.0) ); // Needs a better guess 
    Slvs_hParam p_c1y  = sys.addParam( Slvs_MakeParam(-1, group2, 2.0) ); 
    
    Slvs_hEntity e_C1 = sys.addEntity( Slvs_MakePoint2d(-1, group2, e_Plane1, p_c1x, p_c1y) );

    Slvs_hEntity e_LP1C1 = sys.addEntity(Slvs_MakeLineSegment(-1, group2, e_Plane1, e_P1, e_C1));

    Slvs_hConstraint c_perpT1_LP1C1 = sys.addConstr(Slvs_MakeConstraint(-1,
                                                                        group2,
                                                                        SLVS_C_PERPENDICULAR,
                                                                        e_Plane1,
                                                                        0.0,
                                                                        -1,
                                                                        -1,
                                                                        e_LT1,
                                                                        e_LP1C1));

    Slvs_hConstraint c_dist_P1C1 = sys.addConstr(Slvs_MakeConstraint(-1,
                                                                     group2,
                                                                     SLVS_C_PT_PT_DISTANCE,
                                                                     e_Plane1,
                                                                     rad1,
                                                                     e_P1,
                                                                     e_C1,
                                                                     -1,
                                                                     -1));

    // Arc1 end

    Slvs_hParam p_p11x  = sys.addParam( Slvs_MakeParam(-1, group2, 2.0) ); // Needs a better guess: Perp on p_c1x/p_c1y
    Slvs_hParam p_p11y  = sys.addParam( Slvs_MakeParam(-1, group2, -10.0) ); 

    Slvs_hEntity e_P11 = sys.addEntity( Slvs_MakePoint2d(-1, group2, e_Plane1, p_p11x, p_p11y) );

    Slvs_hEntity e_LC1P11 = sys.addEntity(Slvs_MakeLineSegment(-1, group2, e_Plane1, e_C1, e_P11));


    Slvs_hConstraint c_dist_C1P11 = sys.addConstr(Slvs_MakeConstraint(-1,
                                                                      group2,
                                                                      SLVS_C_EQUAL_LENGTH_LINES,
                                                                      e_Plane1,
                                                                      0.0,
                                                                      -1,
                                                                      -1,
                                                                      e_LP1C1,
                                                                      e_LC1P11));





    // Plane 2

    Slvs_hParam p_Plane2Qw = sys.addParam( Slvs_MakeParam(-1, group2, unitQw) );  
    Slvs_hParam p_Plane2Qx = sys.addParam( Slvs_MakeParam(-1, group2, unitQx) ); 
    Slvs_hParam p_Plane2Qy = sys.addParam( Slvs_MakeParam(-1, group2, unitQy) ); 
    Slvs_hParam p_Plane2Qz = sys.addParam( Slvs_MakeParam(-1, group2, unitQz));
    Slvs_hEntity e_Plane2Q = sys.addEntity( Slvs_MakeNormal3d(-1, group2,
                                                              p_Plane2Qw,
                                                              p_Plane2Qx,
                                                              p_Plane2Qy,
                                                              p_Plane2Qz));

    Slvs_hEntity e_Plane2 = sys.addEntity( Slvs_MakeWorkplane(-1, group2, e_P2, e_Plane2Q) );

    Slvs_hConstraint c_PT2Plane2 = sys.addConstr(Slvs_MakeConstraint(-1,
                                                                     group2,
                                                                     SLVS_C_PT_IN_PLANE,
                                                                     SLVS_FREE_IN_3D,
                                                                     0.0,
                                                                     e_PT2,
                                                                     -1,
                                                                     e_Plane2,
                                                                     -1));

    // Arc2 center

    Slvs_hParam p_c2x  = sys.addParam( Slvs_MakeParam(-1, group2, 10.0) ); // Needs a better guess 
    Slvs_hParam p_c2y  = sys.addParam( Slvs_MakeParam(-1, group2, 2.0) ); 

    Slvs_hEntity e_C2 = sys.addEntity( Slvs_MakePoint2d(-1, group2, e_Plane2, p_c2x, p_c2y) );

    Slvs_hEntity e_LP2C2 = sys.addEntity(Slvs_MakeLineSegment(-1, group2, e_Plane2, e_P2, e_C2));

    Slvs_hConstraint c_perpT2_LP2C2 = sys.addConstr(Slvs_MakeConstraint(-1,
                                                                        group2,
                                                                        SLVS_C_PERPENDICULAR,
                                                                        e_Plane2,
                                                                        0.0,
                                                                        -1,
                                                                        -1,
                                                                        e_LT2,
                                                                        e_LP2C2));

    Slvs_hConstraint c_dist_P2C2 = sys.addConstr(Slvs_MakeConstraint(-1,
                                                                     group2,
                                                                     SLVS_C_PT_PT_DISTANCE,
                                                                     e_Plane2,
                                                                     rad2,
                                                                     e_P2,
                                                                     e_C2,
                                                                     -1,
                                                                     -1));

    // Arc2 end

    Slvs_hParam p_p22x  = sys.addParam( Slvs_MakeParam(-1, group2, 2.0) ); // Needs a better guess: Perp on p_c1x/p_c1y
    Slvs_hParam p_p22y  = sys.addParam( Slvs_MakeParam(-1, group2, -10.0) ); 

    Slvs_hEntity e_P22 = sys.addEntity( Slvs_MakePoint2d(-1, group2, e_Plane2, p_p22x, p_p22y) );

    Slvs_hEntity e_LC2P22 = sys.addEntity(Slvs_MakeLineSegment(-1, group2, e_Plane2, e_C2, e_P22));


    Slvs_hConstraint c_dist_C2P22 = sys.addConstr(Slvs_MakeConstraint(-1,
                                                                      group2,
                                                                      SLVS_C_EQUAL_LENGTH_LINES,
                                                                      e_Plane2,
                                                                      0.0,
                                                                      -1,
                                                                      -1,
                                                                      e_LP2C2,
                                                                      e_LC2P22));



    auto solveResult = sys.solve(group2, true);
    assert(solveResult == SolveSpaceSystem::RESULT_OKAY);

    // Connecting the two planes

    // Connecting line
    Slvs_hEntity e_LP11P22 = sys.addEntity(Slvs_MakeLineSegment(-1, group2, SLVS_FREE_IN_3D, e_P11, e_P22));

    // Perpendicular constraints

    Slvs_hConstraint c_perpC1P11_LP11P22 = sys.addConstr(Slvs_MakeConstraint(-1,
                                                                             group2,
                                                                             SLVS_C_PERPENDICULAR,
                                                                             SLVS_FREE_IN_3D,
                                                                             0.0,
                                                                             -1,
                                                                             -1,
                                                                             e_LC1P11,
                                                                             e_LP11P22));

    solveResult = sys.solve(group2, true);
    assert(solveResult == SolveSpaceSystem::RESULT_OKAY);


    Slvs_hConstraint c_perpC2P22_LP11P22 = sys.addConstr(Slvs_MakeConstraint(-1,
                                                                             group2,
                                                                             SLVS_C_PERPENDICULAR,
                                                                             SLVS_FREE_IN_3D,
                                                                             0.0,
                                                                             -1,
                                                                             -1,
                                                                             e_LC2P22,
                                                                             e_LP11P22));

    solveResult = sys.solve(group2, true);
    assert(solveResult == SolveSpaceSystem::RESULT_OKAY);

    // P11, P22 in plane constraints

    Slvs_hConstraint c_P11InPlane2 = sys.addConstr(Slvs_MakeConstraint(-1,
                                                                     group2,
                                                                     SLVS_C_PT_IN_PLANE,
                                                                     SLVS_FREE_IN_3D,
                                                                     0.0,
                                                                     e_P11,
                                                                     -1,
                                                                     e_Plane2,
                                                                     -1));

    solveResult = sys.solve(group2, true);
    assert(solveResult == SolveSpaceSystem::RESULT_OKAY);

    Slvs_hConstraint c_P22InPlane1 = sys.addConstr(Slvs_MakeConstraint(-1,
                                                                       group2,
                                                                       SLVS_C_PT_IN_PLANE,
                                                                       SLVS_FREE_IN_3D,
                                                                       0.0,
                                                                       e_P22,
                                                                       -1,
                                                                       e_Plane1,
                                                                       -1));


    solveResult = sys.solve(group2, true);
    assert(solveResult == SolveSpaceSystem::RESULT_OKAY);


    // Circle Center, Plane normals, P11, P22

    std::valarray<double> v_C1 = sys.global3DPos(e_C1);
    std::valarray<double> v_C2 = sys.global3DPos(e_C2);

    std::valarray<double> v_N1 = std::get<2>( sys.orientationMx(e_Plane1Q));
    std::valarray<double> v_N2 = std::get<2>( sys.orientationMx(e_Plane2Q));

    std::valarray<double> v_P11 = sys.global3DPos(e_P11);
    std::valarray<double> v_P22 = sys.global3DPos(e_P22);

    std::cout << "C1:  " << "[ " << v_C1[0]  << ", " << v_C1[1]  << ", " << v_C1[2]  << " ]" << std::endl;
    std::cout << "N1:  " << "[ " << v_N1[0]  << ", " << v_N1[1]  << ", " << v_N1[2]  << " ]" << std::endl;
    std::cout << "P11: " << "[ " << v_P11[0] << ", " << v_P11[1] << ", " << v_P11[2] << " ]" << std::endl;
    std::cout << "C2:  " << "[ " << v_C2[0]  << ", " << v_C2[1]  << ", " << v_C2[2]  << " ]" << std::endl;
    std::cout << "N1:  " << "[ " << v_N2[0]  << ", " << v_N2[1]  << ", " << v_N2[2]  << " ]" << std::endl;
    std::cout << "P22: " << "[ " << v_P22[0] << ", " << v_P22[1] << ", " << v_P22[2] << " ]" << std::endl;

    char c;
    std::cin >> c;    
}