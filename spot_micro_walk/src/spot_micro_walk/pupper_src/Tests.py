# using LinearAlgebra
# using Profile
# using StaticArrays
# using Plots
# using BenchmarkTools

# include("Kinematics.jl")
# include("PupperConfig.jl")
# include("Gait.jl")
# include("StanceController.jl")
# include("SwingLegController.jl")
# include("Types.jl")
# include("Controller.jl")

import numpy as np
import matplotlib.pyplot as plt

from Kinematics import leg_explicit_inverse_kinematics
from PupperConfig import *
from Gaits import *
from StanceController import position_delta, stance_foot_location
from SwingLegController import *
from Types import MovementReference, GaitParams, StanceParams, SwingParams
from Controller import *

# function round_(a, dec)
#     return map(x -> round(x, digits=dec), a)
# end

# function testInverseKinematicsExplicit!()
#     println("\n-------------- Testing Inverse Kinematics -----------")
#     config = PupperConfig()
#     println("\nTesting Inverse Kinematics")
#     function testHelper(r, alpha_true, i; do_assert=true)
#         eps = 1e-6
#         @time α = leg_explicitinversekinematics_prismatic(r, i, config)
#         println("Leg ", i, ": r: ", r, " -> α: ", α)
#         if do_assert
#             @assert norm(α - alpha_true) < eps
#         end
#     end

#     c = config.LEG_L/sqrt(2)
#     offset = config.ABDUCTION_OFFSET
#     testHelper(SVector(0, offset, -0.125), SVector(0, 0, 0), 2)
#     testHelper(SVector(c, offset, -c), SVector(0, -pi/4, 0), 2)
#     testHelper(SVector(-c, offset, -c), SVector(0, pi/4, 0), 2)
#     testHelper(SVector(0, c, -c), missing, 2, do_assert=false)

#     testHelper(SVector(-c, -offset, -c), [0, pi/4, 0], 1)
#     testHelper(SVector(config.LEG_L * sqrt(3)/2, offset, -config.LEG_L / 2), SVector(0, -pi/3, 0), 2)
# end


def test_inverse_kinematics_linkage():
    print("\n-------------- Testing Five-bar Linkage Inverse Kinematics -----------")
    config = PupperConfig()
    print("\nTesting Inverse Kinematics")

    def testHelper(r, alpha_true, i, do_assert=True):
        eps = 1e-6
        alpha = leg_explicit_inverse_kinematics(r, i, config)
        print("Leg ", i, ": r: ", r, " -> α: ", alpha)
        if do_assert:
            assert np.linalg.norm(alpha - alpha_true) < eps

    c = config.LEG_L / (2 ** 0.5)
    offset = config.ABDUCTION_OFFSET
    testHelper(np.array([0, offset, -0.125]), None, 1, do_assert=False)
    testHelper(np.array([c, offset, -c]), None, 1, do_assert=False)
    testHelper(np.array([-c, offset, -c]), None, 1, do_assert=False)
    testHelper(np.array([0, c, -c]), None, 1, do_assert=False)

    testHelper(np.array([-c, -offset, -c]), None, 0, do_assert=False)
    testHelper(
        np.array([config.LEG_L * (3 ** 0.5) / 2, offset, -config.LEG_L / 2]),
        None,
        1,
        do_assert=False,
    )


# function testForwardKinematics!()
#     println("\n-------------- Testing Forward Kinematics -----------")
#     config = PupperConfig()
#     println("\nTesting Forward Kinematics")
#     function testHelper(alpha, r_true, i; do_assert=true)
#         eps = 1e-6
#         r = zeros(3)
#         println("Vectors")
#         a = [alpha.data...]
#         @time legForwardKinematics!(r, a, i, config)
#         println("SVectors")
#         @time r = legForwardKinematics(alpha, i, config)
#         println("Leg ", i, ": α: ", alpha, " -> r: ", r)
#         if do_assert
#             @assert norm(r_true - r) < eps
#         end
#     end

#     l = config.LEG_L
#     offset = config.ABDUCTION_OFFSET
#     testHelper(SVector{3}([0.0, 0.0, 0.0]), SVector{3}([0, offset, -l]), 2)
#     testHelper(SVector{3}([0.0, pi/4, 0.0]), missing, 2, do_assert=false)
#     # testHelper([0.0, 0.0, 0.0], [0, offset, -l], 2)
#     # testHelper([0.0, pi/4, 0.0], missing, 2, do_assert=false)
# end

# function testForwardInverseAgreeance()
#     println("\n-------------- Testing Forward/Inverse Consistency -----------")
#     config = PupperConfig()
#     println("\nTest forward/inverse consistency")
#     eps = 1e-6
#     for i in 1:10
#         alpha = SVector(rand()-0.5, rand()-0.5, (rand()-0.5)*0.05)
#         leg = rand(1:4)
#         @time r = legForwardKinematics(alpha, leg, config)
#         # @code_warntype legForwardKinematics!(r, alpha, leg, config)
#         @time alpha_prime = leg_explicitinversekinematics_prismatic(r, leg, config)
#         # @code_warntype inverseKinematicsExplicit!(alpha_prime, r, leg, config)
#         println("Leg ", leg, ": α: ", round_(alpha, 3), " -> r_body_foot: ", round_(r, 3), " -> α': ", round_(alpha_prime, 3))
#         @assert norm(alpha_prime - alpha) < eps
#     end
# end

# function testAllInverseKinematics()
#     println("\n-------------- Testing Four Leg Inverse Kinematics -----------")
#     function helper(r_body, alpha_true; do_assert=true)
#         println("Timing for fourlegs_inversekinematics")
#         config = PupperConfig()
#         @time alpha = fourlegs_inversekinematics(SMatrix(r_body), config)
#         @code_warntype fourlegs_inversekinematics(SMatrix(r_body), config)
#         println("r: ", r_body, " -> α: ", alpha)

#         if do_assert
#             @assert norm(alpha - alpha_true) < 1e-10
#         end
#     end
#     config = PupperConfig()
#     f = config.LEG_FB
#     l = config.LEG_LR
#     s = -0.125
#     o = config.ABDUCTION_OFFSET
#     r_body = MMatrix{3,4}(zeros(3,4))
#     r_body[:,1] = [f, -l-o, s]
#     r_body[:,2] = [f, l+o, s]
#     r_body[:,3] = [-f, -l-o, s]
#     r_body[:,4] = [-f, l+o, s]

#     helper(r_body, zeros(3,4))
#     helper(SMatrix{3,4}(zeros(3,4)), missing, do_assert=false)
# end

# function testKinematics()
#     testInverseKinematicsExplicit!()
#     testForwardKinematics!()
#     testForwardInverseAgreeance()
#     testAllInverseKinematics()
# end

# function testGait()
#     println("\n-------------- Testing Gait -----------")
#     p = GaitParams()
#     # println("Gait params=",p)
#     t = 680
#     println("Timing for phaseindex")
#     @time ph = phaseindex(t, p)
#     # @code_warntype phaseindex(t, p)
#     println("t=",t," phase=",ph)
#     @assert ph == 4
#     @assert phaseindex(0, p) == 1

#     println("Timing for contacts")
#     @time c = contacts(t, p)
#     # @code_warntype contacts(t, p)
#     @assert typeof(c) == SArray{Tuple{4},Int64,1,4}
#     println("t=", t, " contacts=", c)
# end


def test_stance_controller():
    print("\n-------------- Testing Stance Controller -----------")
    stanceparams = StanceParams()
    gaitparams = GaitParams()

    zmeas = -0.20
    mvref = MovementReference()
    dp, dR = position_delta(zmeas, stanceparams, mvref, gaitparams)
    assert np.linalg.norm(dR - np.eye(3)) < 1e-10
    assert np.linalg.norm(dp - np.array([0, 0, gaitparams.dt * 0.04])) < 1e-10

    zmeas = -0.18
    mvref = MovementReference()
    mvref.v_xy_ref = np.array([1.0, 0.0])
    mvref.z_ref = -0.18
    dp, dR = position_delta(zmeas, stanceparams, mvref, gaitparams)

    zmeas = -0.20
    mvref = MovementReference()
    mvref.wz_ref = 1.0
    mvref.z_ref = -0.20
    dp, dR = position_delta(zmeas, stanceparams, mvref, gaitparams)
    assert np.linalg.norm(dp - np.array([0, 0, 0])) < 1e-10
    assert np.linalg.norm(dR[0, 1] - (gaitparams.dt)) < 1e-6

    stancefootloc = np.zeros(3)
    sloc = stance_foot_location(stancefootloc, stanceparams, gaitparams, mvref)


# function typeswinglegcontroller()
#     println("\n--------------- Code warn type for raibert_tdlocation[s] ----------")
#     swp = SwingParams()
#     stp = StanceParams()
#     gp = GaitParams()
#     mvref = MovementReference(SVector(1.0, 0.0), 0, -0.18)
#     raibert_tdlocations(swp, stp, gp, mvref)

#     mvref = MovementReference(SVector(1.0, 0.0), 0, -0.18)
#     raibert_tdlocation(1, swp, stp, gp, mvref)
# end

# function TestSwingLegController()
#     println("\n-------------- Testing Swing Leg Controller -----------")
#     swp = SwingParams()
#     stp = StanceParams()
#     gp = GaitParams()
#     p = ControllerParams()
#     println("Timing for swingheight:")
#     @time z = swingheight(0.5, swp)
#     println("z clearance at t=1/2swingtime =>",z)
#     @assert abs(z - swp.zclearance) < 1e-10

#     println("Timing for swingheight:")
#     @time z = swingheight(0, swp)
#     println("Z clearance at t=0 =>",z)
#     @assert abs(z) < 1e-10

#     mvref = MovementReference(SVector(1.0, 0.0), 0, -0.18)
#     println("Timing for raibert tdlocation*s*:")
#     @time l = raibert_tdlocations(swp, stp, gp, mvref)
#     target = stp.defaultstance .+ [gp.stanceticks*gp.dt*0.5*1, 0, 0]
#     println("Touchdown locations =>", l, " <?=> ", target)
#     @assert norm(l - target) <= 1e-10

#     mvref = MovementReference(SVector(1.0, 0.0), 0, -0.18)
#     println("Timing for raibert tdlocation:")
#     @time l = raibert_tdlocation(1, swp, stp, gp, mvref)

#     fcurrent = SMatrix{3, 4, Float64}(stp.defaultstance)
#     mvref = MovementReference()
#     tswing = 0.125
#     println("Timing for swingfootlocation*s* increment")
#     @time l = swingfootlocations(tswing, fcurrent, swp, stp, gp, mvref)
#     println(l)

#     fcurrent = SVector{3, Float64}(0.0, 0.0, 0.0)
#     println("Timing for swingfootlocation")
#     @time swingfootlocation(tswing, fcurrent, 1, swp, stp, gp, mvref)

#     typeswinglegcontroller()
#     return nothing
# end


def test_run():
    print("Run timing")
    foot_loc_history, joint_angle_history = run()
    plt.subplot(211)
    x = plt.plot(foot_loc_history[0, :, :].T, label="x")
    y = plt.plot(foot_loc_history[1, :, :].T, label="y")
    z = plt.plot(foot_loc_history[2, :, :].T, label="z")

    plt.subplot(212)
    alpha = plt.plot(joint_angle_history[0, :, :].T, label="alpha")
    beta = plt.plot(joint_angle_history[1, :, :].T, label="beta")
    gamma = plt.plot(joint_angle_history[2, :, :].T, label="gamma")
    plt.show()

    # plot(x, β, y, α, z, γ, layout=(3,2), legend=false))


# function teststep()
#     swingparams = SwingParams()
#     stanceparams = StanceParams()
#     gaitparams = GaitParams()
#     mvref = MovementReference(vxyref=SVector{2}(0.2, 0.0), wzref=0.0)
#     conparams = ControllerParams()
#     robotconfig = PupperConfig()


#     footlocations::SMatrix{3, 4, Float64, 12} = stanceparams.defaultstance .+ SVector{3, Float64}(0, 0, mvref.zref)

#     ticks = 1
#     println("Timing for step!")
#     @btime step($ticks, $footlocations, $swingparams, $stanceparams, $gaitparams, $mvref, $conparams)
#     @code_warntype step(ticks, footlocations, swingparams, stanceparams, gaitparams, mvref, conparams)
# end

# # testGait()
# # testKinematics()
# # TestStanceController()
# # testStaticArrays()
# # TestSwingLegController()
# test_inversekinematics_linkage()

# # teststep()
# # testrun()

test_inverse_kinematics_linkage()
test_stance_controller()
test_run()
