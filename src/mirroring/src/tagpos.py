#!/usr/bin/env python
import rospy
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from moveit_msgs.msg import Constraints
from geometry_msgs.msg import PoseStamped, TransformStamped, Transform, Vector3
from moveit_commander import MoveGroupCommander
import numpy as np
from numpy import linalg
import sys

import sys
import argparse
import numpy as np
import rospkg
import roslaunch

from paths.trajectories import LinearTrajectory, CircularTrajectory
from paths.paths import MotionPath
from paths.path_planner import PathPlanner
from controllers.controllers import ( 
    PIDJointVelocityController, 
    FeedforwardJointVelocityController
)
from utils.utils import *

from trac_ik_python.trac_ik import IK

import rospy
import tf2_ros
import intera_interface
from moveit_msgs.msg import DisplayTrajectory, RobotState
from sawyer_pykdl import sawyer_kinematics

from tf2_geometry_msgs import do_transform_pose


def tuck():
    """
    Tuck the robot arm to the start position. Use with caution
    """
    if input('Would you like to tuck the arm? (y/n): ') == 'y':
        rospack = rospkg.RosPack()
        path = rospack.get_path('sawyer_full_stack')
        launch_path = path + '/launch/custom_sawyer_tuck.launch'
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch = roslaunch.parent.ROSLaunchParent(uuid, [launch_path])
        launch.start()
    else:
        print('Canceled. Not tucking the arm.')

def lookup_tag(tag_number):
    """
    Given an AR tag number, this returns the position of the AR tag in the robot's base frame.
    You can use either this function or try starting the scripts/tag_pub.py script.  More info
    about that script is in that file.  

    Parametersar_pos
    ----------
    tag_number : int

    Returns
    -------
    3x' :obj:`numpy.ndarray`
        tag position
    """

    
    # TODO: initialize a tf buffer and listener as in lab 3
    
    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)

    try:
        # TODO: lookup the transform and save it in trans

        # The rospy.Time(0) is the latest available 
        # The rospy.Duration(10.0) is the amount of time to wait for the transform to be available before throwing an exception
        trans = tfBuffer.lookup_transform("base", f"ar_marker_{tag_number}", rospy.Time(0), rospy.Duration(10.0))
        #trans_inv = tfBuffer.lookup_transform(f"ar_marker_{tag_number}", "base",  rospy.Time(0), rospy.Duration(10.0))
    except Exception as e:
        print(e)
        print("Retrying ...")

    tag_pos = [getattr(trans.transform.translation, dim) for dim in ('x', 'y', 'z')]
    return np.array(tag_pos), trans#, trans_inv

def main():
    # Wait for the IK service to become available
    #rospy.wait_for_service('compute_ik')
    #rospy.init_node('service_query')
    # Create the function used to call the service
    #compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)


    #Get the transform 
    ar_pos, ar_trans = lookup_tag(5)
    print(ar_trans)
    print("------------------")
    sys.exit()
    
    # manually writing out the transform
    ar_trans = TransformStamped()
    ar_trans.header.frame_id = "base"
    ar_trans.child_frame_id = "ar_marker_5"


    ar_trans.transform.translation.x = 0.9168308570235706
    ar_trans.transform.translation.y = 0.7110835508043919
    ar_trans.transform.translation.z = 0.35409153525141457
    ar_trans.transform.rotation.x = -0.5372949769275619
    ar_trans.transform.rotation.y = -0.5311668177057733
    ar_trans.transform.rotation.z = 0.47656178354926226
    ar_trans.transform.rotation.w = 0.4495161687827037


    ar_trans_inv = TransformStamped()
    ar_trans_inv.header.frame_id = "ar_marker_5"
    ar_trans_inv.child_frame_id = "base"


    ar_trans_inv.transform.translation.x = 0.022324429215888314
    ar_trans_inv.transform.translation.y =-0.43970142756613806
    ar_trans_inv.transform.translation.z = 1.248374854610291
    ar_trans_inv.transform.rotation.x = -0.5344219637075072
    ar_trans_inv.transform.rotation.y = 0.41073510873129104
    ar_trans_inv.transform.rotation.z = 0.4669217745661795
    ar_trans_inv.transform.rotation.w = 0.5724280667459246
    # print(ar_pos)
    #print("transform: \n", ar_trans)

    #manually set constraints
    #moveit_constraints = Constraints()

    #tuck()

    hand_poses_raw = [
        [0.14838534326720496, -0.0023975380886044196, 0.659000000246855],
[0.14838534326720496, -0.001085781336871934, 0.6595000002471159],
[0.14912617258102046, -0.0005603449382040073, 0.6600000002367221],
[0.14968632268676743, 0.00011494265524475803, 0.6600000002367221],
[0.15013895410130845, 0.0006154630145759125, 0.6605000002315783],
[0.15093411537305998, 0.001422235192133794, 0.6605000002315783],
[0.15166076795721373, 0.0022344337939675853, 0.6615000002359104],
[0.15179436395601575, 0.0022803464565458666, 0.6615000002391029],
[0.15179436395601575, 0.0022803464565458666, 0.6615000002391029],
[0.1512792015060166, 0.0018414132663895195, 0.6595000002436254],
[0.15024069877531032, 0.0016021482061055006, 0.657500000243354],
[0.14906214383268815, 0.0016021482061055006, 0.6565000002422245],
[0.14782648664335932, 0.0017322210243822089, 0.6560000002413833],
[0.1462279712402217, 0.0020831743703026966, 0.6555000002380214],
[0.14440014764110473, 0.0023572227251143583, 0.6545000002330186],
[0.14256886104353528, 0.0025321843127854215, 0.654000000229222],
[0.14118438892227836, 0.002682811513667354, 0.65350000023338],
[0.14033797831470174, 0.0028467777970545855, 0.6530000002367528],
[0.13946878296386686, 0.0030995357414755936, 0.6515000002391708],
[0.13873723655565998, 0.003445236852177913, 0.6500000002414394],
[0.13763985652993985, 0.003592713360501454, 0.64900000024119],
[0.13671636718236363, 0.0037266827562408367, 0.6470000002397348],
[0.13633378262205714, 0.0037266827562408367, 0.6455000002453986],
[0.13599334500479324, 0.0037937460855065313, 0.6450000002480385],
[0.13524049464005647, 0.0037937460855065313, 0.6435000002488837],
[0.13419362321123487, 0.003775475215037277, 0.6420000002530956],
[0.13374298559902628, 0.0036735816054240932, 0.6420000002499134],
[0.13351597364749876, 0.0034334646362654615, 0.6410000002525303],
[0.13320534327848396, 0.0027571451843312186, 0.640000000257799],
[0.13301654311868505, 0.0023303371302705288, 0.640000000257799],
[0.13294364133870165, 0.002197703341591678, 0.6400000002559182],
[0.13294364133870165, 0.001785298421615472, 0.6395000002520819],
[0.13318778044493132, 0.0014644860832615288, 0.6395000002476224],
[0.1340464530579449, 0.0014644860832615288, 0.6395000002468206],
[0.13461514757420906, 0.0017114704818252861, 0.6400000002457366],
[0.13535287936698837, 0.002139601014410633, 0.6410000002488432],
[0.13679522251797677, 0.002603988931132058, 0.6425000002423326],
[0.13788527295157332, 0.003524685598159068, 0.6430000002314655],
[0.13848401374590683, 0.004375222194197237, 0.6435000002317727],
[0.138909169493958, 0.0047096882115887926, 0.6440000002320272],
[0.13924386451245344, 0.005105243191742126, 0.6450000002322599],
[0.1394523793046132, 0.0056914657623409115, 0.6460000002325368],
[0.13979830855898798, 0.0060240454710457705, 0.6465000002353899],
[0.1394523793046132, 0.006208530487540491, 0.6470000002371857],
[0.13948062400777786, 0.006490201444838446, 0.6470000002371857],
[0.13958892916899476, 0.006847990102915929, 0.6475000002360849],
[0.1395670737935466, 0.007266088650044888, 0.6475000002360849],
[0.1397061090661172, 0.007481277238378386, 0.64800000023456],
[0.1399993148901247, 0.007573216827731711, 0.64800000023456],
[0.1402958208240486, 0.007724643541830031, 0.6485000002335047],
[0.14087885114190865, 0.007940284754193146, 0.6495000002319558],
[0.14191433157273, 0.008417179691470293, 0.650000000232096],
[0.14250056574268843, 0.008817189655167777, 0.6515000002289094],
[0.14294334612514611, 0.009079062995833852, 0.6525000002267113],
[0.1439471915517088, 0.009431763883352484, 0.6530000002282077],
[0.14469487656931107, 0.010036207162038537, 0.6530000002329882],
[0.14511014237884073, 0.010734407187441956, 0.6535000002312809],
[0.1455432669262775, 0.01133865548658811, 0.6545000002288491],
[0.1457814291412446, 0.011832749078287836, 0.655000000229642],
[0.1457814291412446, 0.012118581378873993, 0.655000000229642],
[0.1459748080218135, 0.012505977909291992, 0.6545000002282261],
[0.1457423240787229, 0.012886806980128502, 0.6540000002254671],
[0.14558818024318565, 0.013031243727131325, 0.6540000002254671],
[0.14558818024318565, 0.013081890856104936, 0.6545000002287288],
[0.14549751718120899, 0.013597560002108516, 0.6540000002275911],
[0.14501832993062014, 0.014368675228140277, 0.6540000002286336],
[0.14442415443436432, 0.014769061732945002, 0.6535000002307957],
[0.14415256261946371, 0.015100369083024241, 0.6535000002307957],
[0.14368411487156146, 0.015631829740388643, 0.6535000002307957],
[0.14227703040040698, 0.01605028998363271, 0.6525000002271093],
[0.13999748650153634, 0.016376783285179157, 0.6495000002238489],
[0.13847891335448465, 0.016836334002025274, 0.6490000002233667],
[0.13754561545094082, 0.017363869705491235, 0.6485000002279844],
[0.13640281597594459, 0.018210554249287997, 0.6495000002258631],
[0.13499819477700528, 0.019050811470713358, 0.650000000222666],
[0.1332539245740052, 0.02010815557493978, 0.6510000002241606],
[0.1315608945504457, 0.021363013586254996, 0.6510000002291388],
[0.12914016402493106, 0.021902232289598627, 0.6510000002291388],
[0.1260430478941595, 0.02213083002681487, 0.650000000229149],
[0.12359606589871015, 0.022789922675290435, 0.649000000226604],
[0.12115558988623028, 0.023387952451994033, 0.6490000002141298],
[0.11816451936569533, 0.023879394292014498, 0.6500000001980534],
[0.11605388095868273, 0.02460139273826254, 0.6490000002042168],
[0.11416781772935311, 0.02500649749430108, 0.6490000002010432],
[0.11151380008737402, 0.025169930291612887, 0.6480000002073592],
[0.10915415622461475, 0.025278754005536825, 0.6480000002073592],
[0.10737651885551287, 0.025551473376925166, 0.6490000002100196],
[0.10532604241928933, 0.025892040120061872, 0.6495000002145567],
[0.10313040558165448, 0.026108233465998852, 0.6495000002145567],
[0.10119281078439343, 0.02618710173542008, 0.6490000002179779],
[0.09886807380207409, 0.026301843807071505, 0.6485000002183351],
[0.09621794305610351, 0.026388923343621726, 0.6485000002151422],
[0.09443962096857848, 0.026319829680662026, 0.6485000002151422],
[0.09263904941908128, 0.026178851428439658, 0.6485000002132274],
[0.09009232249362657, 0.025715074052610018, 0.6485000002104336],
[0.08831200944606289, 0.025511213557298715, 0.6475000002097382],
[0.08656676755863271, 0.025452796633696793, 0.6465000002118809],
[0.08456720293270646, 0.025350757742439084, 0.6465000002113078],
[0.08254445098154002, 0.02510168119358799, 0.6465000002103961],
[0.08093455187582135, 0.025001525956109442, 0.6465000002108441],
[0.07984365083328035, 0.024853180906433858, 0.6465000002108441],
[0.07839681482718008, 0.02476233609195449, 0.6460000002108464],
[0.07708805159855442, 0.024307224418761778, 0.6460000002086616],
[0.0757974029064269, 0.02364183528756569, 0.6455000002082533],
[0.0745818971943263, 0.02335004304335144, 0.6445000002102427],
[0.07307637959463587, 0.02274104546139226, 0.6435000002072009],
[0.07144142823943841, 0.021958665826823837, 0.6415000002081553],
[0.06981566644025117, 0.02107440948032598, 0.6400000002037638],
[0.0678715992282459, 0.01963540530050075, 0.6385000001954332],
[0.06623654378563427, 0.0181997472763853, 0.6380000001946872],
[0.06494934836557358, 0.017072523020190078, 0.6370000001955187],
[0.06332843403273612, 0.016193273937975313, 0.6360000001997892],
[0.06151965045434952, 0.01502706808105173, 0.6360000001943599],
[0.05983757786743478, 0.0136360472269405, 0.6355000001959684],
[0.057937347284489835, 0.012065269189558709, 0.6345000001966694],
[0.056639460710028296, 0.010360634498767545, 0.6335000001923118],
[0.05563354499395976, 0.008785687693269683, 0.6330000001886026],
[0.05436594786457502, 0.007106483966080504, 0.6325000001909213],
[0.05313788683237378, 0.005283160800471677, 0.631500000197674],
[0.05217895753392592, 0.0034746969678375707, 0.631500000197674],
[0.05139128252135444, 0.0018096643186477598, 0.6315000002021846],
[0.0503117204955642, -4.942263800092467e-05, 0.6315000002021415],
[0.0496746965079541, -0.0018684120302789244, 0.632500000199831],
[0.049389017489599084, -0.0032312187327388693, 0.6320000001971027],
[0.04839958782408859, -0.004512817085846964, 0.631000000198185],
[0.04749094751537325, -0.005694764045148238, 0.630500000196916],
[0.047036238263901485, -0.006481690273779911, 0.6305000001946719],
[0.046466207218658594, -0.007053537693100294, 0.6300000001965921],
[0.045625819388263336, -0.007768943356767438, 0.6295000001926192],
[0.04492916849149992, -0.008734601542972462, 0.6295000001901544],
[0.044724716433735864, -0.0095597664649865, 0.6295000001901544],
[0.04426737628481213, -0.010143706025990006, 0.6305000001887238],
[0.043892540475630785, -0.010858756251962474, 0.6305000001887238],
[0.04377164872928773, -0.011656985165266222, 0.6305000001880621],
[0.043471888224279824, -0.0123229255196729, 0.6305000001876852],
[0.04337440811544788, -0.012874596211613045, 0.6300000001876828],
[0.0432365080802707, -0.01346613248300519, 0.6305000001876852],
[0.04330086968271805, -0.014134970351632064, 0.6310000001867755],
[0.04351883785074458, -0.01479266622155852, 0.6310000001862468],
[0.04378143707384626, -0.015435089733391187, 0.6315000001869246],
[0.04410826482141136, -0.015879998662510813, 0.6320000001869753],
[0.04429272165992177, -0.016316271785277267, 0.633000000186244],
[0.04452444730401078, -0.017138101985288565, 0.6330000001882443],
[0.04482188052132275, -0.01806277882760975, 0.6330000001882443],
[0.044867398058560326, -0.018466997466596833, 0.633000000188539],
[0.0450181758145211, -0.01891932307356983, 0.6330000001912593],
[0.04532395275027812, -0.01963203660019779, 0.6330000001912593],
[0.04588589543369367, -0.020386924700333642, 0.6330000001912593],
[0.04646386638360929, -0.02118286123539781, 0.6325000001964078],
[0.04705852031675454, -0.02165817644541407, 0.6320000001975589],
[0.047461123497402315, -0.02198154328235711, 0.6315000001992758],
[0.0477977985815216, -0.022483312246733427, 0.6315000001999433],
[0.048167837927096016, -0.02294371338364872, 0.6310000002004279],
[0.048564643778785124, -0.023343498592749676, 0.6315000001997713],
[0.049052902918861795, -0.023934422867579953, 0.6315000001996238],
[0.049471429173733386, -0.02471352983609122, 0.6315000001996238],
[0.04987788849215453, -0.025669671857023766, 0.6325000001954055],
[0.05084318902238127, -0.02647082263315692, 0.6330000001951221],
[0.05194618327625875, -0.027391741888125046, 0.6330000002030493],
[0.052707267683624895, -0.028552529307772447, 0.6330000002107342],
[0.053430152364217734, -0.029452297118570586, 0.6330000002107342],
[0.05427075752782335, -0.030462404929617118, 0.6325000002094538],
[0.055001206879171816, -0.031811884962169926, 0.6325000002099348],
[0.055817264157166514, -0.03283664592327617, 0.6320000002091098],
[0.056761583225054195, -0.033502941819685335, 0.6325000002099348],
[0.0576540824632209, -0.034145995150556316, 0.6330000002086679],
[0.05853971049502667, -0.034862427796367834, 0.633000000210062],
[0.05940554352476716, -0.03589736876488058, 0.6330000002139877],
[0.060369227949481304, -0.03702691353661304, 0.633000000210062],
[0.061356887619953594, -0.0376835231637313, 0.6325000002170036],
[0.06209863468226762, -0.03833691945912724, 0.6325000002184582],
[0.06273783216251073, -0.039067546424615913, 0.6325000002185605],
[0.06339355545237563, -0.0402224046615534, 0.6330000002172762],
[0.06449274090578891, -0.04183022995704718, 0.6335000002147625],
[0.06605586858981173, -0.043471659420430346, 0.6345000002093878],
[0.06768998392734292, -0.04529916359603682, 0.6355000002067758],
[0.06959861061907409, -0.046883002771486645, 0.6360000002056613],
[0.0722083077646027, -0.048156493747961215, 0.6370000002068543],
[0.07452537958822936, -0.04903395284490853, 0.6390000002068721],
[0.07655765565401493, -0.04979462476708392, 0.6390000002097683],
[0.07878985643897488, -0.05040996181144679, 0.6390000002097683],
[0.0808240700644373, -0.05092135800517502, 0.6405000002097356],
[0.08267377038586886, -0.051794339379285216, 0.6415000002125011],
[0.0846127632609801, -0.052227737990428645, 0.6430000002116573],
[0.08699150212526252, -0.05233313369778723, 0.6455000002189192],
[0.08906910773376453, -0.05239275586312199, 0.647500000221831],
[0.09073785087960996, -0.052352684949391604, 0.6480000002182171],
[0.09254996437861979, -0.05239556704833164, 0.6480000002197474],
[0.0944800723497174, -0.052456990682197266, 0.6480000002197474],
[0.09567655184319404, -0.05272237887729092, 0.6480000002215062],
[0.09662705604233547, -0.05335300611620926, 0.6480000002231388],
[0.0982343131343164, -0.05444622724474506, 0.6485000002261433],
[0.09952033384235663, -0.055919735885924346, 0.6495000002285526],
[0.10104531179180254, -0.057374439001937304, 0.6515000002330011],
[0.1034078951846883, -0.05871827503682641, 0.6530000002376757],
[0.10548734967534512, -0.06040971140151463, 0.6530000002376757],
[0.10768356076523918, -0.0619546137580688, 0.6525000002493501],
[0.10958387162555393, -0.06311207455195675, 0.6525000002529484],
[0.11129248912334086, -0.06414017501707801, 0.6525000002591991],
[0.11351649205817535, -0.06488100308651347, 0.6540000002618224],
[0.11516888873831829, -0.06527242284067052, 0.6560000002645693],
[0.11648572627588434, -0.06527242284067052, 0.6565000002652439],
[0.11772048731976623, -0.06499772060760092, 0.6560000002698776],
[0.11848537057957079, -0.064635973328288, 0.6565000002679636],
[0.11930049318498988, -0.06392351802499902, 0.657000000272713],
[0.12026033492354267, -0.06332274527434854, 0.6585000002670564],
[0.12108949091794, -0.06332274527434854, 0.6590000002628482],
[0.12228649201679113, -0.06359835707623744, 0.6590000002628482],
[0.12364590137314765, -0.06437569835355648, 0.659500000256783],
[0.12533308392016412, -0.06541186871854501, 0.6595000002586301],
[0.12693157251163745, -0.06595264317970546, 0.6595000002642807],
[0.12837779141325983, -0.06595264317970546, 0.6605000002638359],
[0.1301289756754551, -0.06587689183062602, 0.6615000002596518],
[0.1310507988931916, -0.06539552586715132, 0.662000000258724],
[0.1316574447920291, -0.06437003670148651, 0.6620000002611593],
[0.13253534854785415, -0.06340304680092874, 0.6620000002576505],
[0.13339133184092514, -0.06241716487042857, 0.661000000256767],
[0.13499565019263582, -0.061693283822696274, 0.6610000002558897],
[0.1381615335566795, -0.061511055072662746, 0.6605000002536004],
[0.14187287371088872, -0.06111254727153444, 0.6605000002536004],
[0.14574514676568157, -0.06025975327984035, 0.6600000002452582],
[0.15113113621777824, -0.059151515353547214, 0.6590000002360866],
[0.15722585521976812, -0.057973361960480835, 0.6590000002360866],
[0.16337723703482807, -0.05647719234987028, 0.6590000002392143],
[0.16890205263951436, -0.05413203004768571, 0.6590000002385378],
[0.1737217468979933, -0.051551875969299436, 0.6595000002426255],
[0.1798667361637706, -0.04910540926265729, 0.6595000002426255],
[0.18637530176979789, -0.04634278635330492, 0.6605000002470609],
[0.1922178466109561, -0.04321874221641221, 0.6615000002418401],
[0.19720991681638322, -0.039408393661601156, 0.6620000002424841],
[0.201163493257776, -0.0346842377020949, 0.6620000002538192],
[0.20426990887702795, -0.029280764188099415, 0.6620000002538192],
[0.205659834515306, -0.023868850392419552, 0.662000000256473],
[0.20583737498815613, -0.0182104456278934, 0.6605000002582004],
[0.20581503328186876, -0.012646154965578084, 0.6605000002573098],
[0.20581503328186876, -0.007069708434367598, 0.6610000002516971],
[0.20362552962348332, -0.0012603694653238766, 0.6600000002464805],
[0.20045042992150128, 0.004012334827516454, 0.6600000002464805],
[0.19701312221104586, 0.008837783158076491, 0.6595000002471911],
[0.19193507664870793, 0.01390299460834347, 0.6595000002471911],
[0.18728058345511794, 0.019525228457551676, 0.6580000002469005],
[0.18203535072894686, 0.0245420064993744, 0.6570000002426909],
[0.1759654348081706, 0.028480747220905527, 0.6570000002356637],
[0.16911264482409832, 0.031409793744330394, 0.6550000002274392],
[0.16160332632340693, 0.03406703757570463, 0.65250000023097],
[0.1549332084371065, 0.03640855386232475, 0.65100000024472],
[0.14866210011728623, 0.03756628395685134, 0.6500000002467531],
[0.14117398966783878, 0.037998169299176994, 0.6480000002508614],
[0.13206098599198274, 0.03835122907029328, 0.6450000002461099],
[0.12356010361420343, 0.03835122907029328, 0.6440000002235775],
[0.1154471968524858, 0.03795728107607699, 0.6420000002175587],
[0.1065498975367855, 0.03736513878289574, 0.6385000002271995],
[0.09833012522193275, 0.03736513878289574, 0.636500000230451],
[0.09018416898837914, 0.03786715592849591, 0.6355000002342425],
[0.08203468207956532, 0.03786715592849591, 0.6340000002337041],
[0.07481012140898521, 0.037602923855164866, 0.6330000002251713],
[0.06725132994497679, 0.03660034889880552, 0.6310000002162044],
[0.05983709737844444, 0.03616468582291219, 0.6285000001969856],
[0.052137042452468106, 0.035731024600547064, 0.6265000001943967],
[0.044459436069314254, 0.03528014210428009, 0.6245000001934725],
[0.03745452577009247, 0.03469638273424584, 0.62350000018982],
[0.03029977283034841, 0.032920590616474926, 0.6225000001901986],
[0.023306889834113356, 0.029737070596274173, 0.6210000001823796],
[0.016870358156634027, 0.025709784554965903, 0.6200000001842089],
[0.01110921895710084, 0.021858951222027277, 0.6195000001807193],
[0.005414742878367852, 0.017908942778036327, 0.6180000001851603],
[1.4864070524653883e-06, 0.01306052044592952, 0.6155000001901721],
[-0.005084240324979229, 0.006911409781373994, 0.6125000002020836],
[-0.01051414636834336, 0.00036243787193633657, 0.6110000002057918],
[-0.016321101347160565, -0.006608610323136772, 0.6105000002101171],
[-0.021741929501117318, -0.014757258561662774, 0.6075000002249583],
[-0.026355875438097018, -0.023460969296308126, 0.6050000002220154],
[-0.030531923400550325, -0.03183506489429503, 0.6040000002079926],
[-0.03363471204026365, -0.04016824535253591, 0.6040000001958424],
[-0.03610241431408026, -0.04850061187396922, 0.6025000001960901],
[-0.03807423230116132, -0.0563425445700899, 0.6035000001924807],
[-0.039527466413434964, -0.0635206609399052, 0.6040000001760103],
[-0.04036006289013241, -0.06980655620492175, 0.6040000001760103],
[-0.04036006289013241, -0.07531640384432178, 0.6045000001776097],
[-0.04024456623771942, -0.08029139600797948, 0.6050000001773279],
[-0.03923641079041032, -0.08457124666284584, 0.6055000001734341],
[-0.037692699338245136, -0.08873319428137313, 0.6050000001773279],
[-0.0356703021246923, -0.09216037986454632, 0.6050000001741498],
[-0.0336959304680477, -0.09501518692437444, 0.6055000001762108],
[-0.03183742022900861, -0.09836920324169748, 0.6065000001702001],
[-0.029007595160607715, -0.10159737325263857, 0.607500000187598],
[-0.026205860132188136, -0.10388680220554465, 0.6085000001745575],
[-0.023534493389960297, -0.10591482997917964, 0.6100000001927398],
[-0.02062254958960205, -0.10786796153438627, 0.611000000222967],
[-0.017403759933767467, -0.10929477171086432, 0.6115000002213565],
[-0.013936007310427935, -0.11030989452611038, 0.6125000002211523],
[-0.010398439867083508, -0.11095009393581022, 0.6135000002057454],
[-0.0065327116849644064, -0.11208258118682451, 0.6145000001782033],
[-0.00270352858043695, -0.11386382692504497, 0.6160000001771444],
[0.0015766997860430947, -0.11580126021295727, 0.6170000001920972],
[0.005948358971819804, -0.11825909922892347, 0.6180000002161146],
[0.010373212822537642, -0.12079317617667006, 0.6190000002241511],
[0.014711620329625058, -0.12267534955152885, 0.6195000002293718],
[0.01890583065658425, -0.12451560106509658, 0.6210000002391075],
[0.0238172735397987, -0.12622478704919105, 0.623000000242661],
[0.028642448916729464, -0.12762449723624705, 0.6240000002424765],
[0.03336602353181316, -0.12900803789262888, 0.6245000002240646],
[0.038141327437181335, -0.12996061823212113, 0.6250000002108763],
[0.042876572312839804, -0.13113297168707697, 0.6255000002050569],
[0.047510131192344617, -0.13245314555447538, 0.6265000001877294],
[0.05220829199811676, -0.13315643378730835, 0.6265000001877294],
[0.057422168063659546, -0.1335640255752208, 0.6265000001877294],
[0.06278677085218885, -0.13383124411051278, 0.6275000001864671],
[0.06847753814462397, -0.1340568695703541, 0.6285000001860159],
[0.07426139011592511, -0.1340568695703541, 0.6290000001889373],
[0.07940858130288118, -0.13410213406851682, 0.6295000001873026],
[0.08461636322236794, -0.13388587660502646, 0.6305000001975801],
[0.08996190259205114, -0.13360008014185715, 0.6305000002170681],
[0.09486213524080611, -0.13348644338694798, 0.6310000002187774],
[0.0999222910858301, -0.13316844090550278, 0.6310000002140969],
[0.1051268879389362, -0.13274536985087484, 0.6315000002193665],
[0.10998387292610182, -0.13249408091709172, 0.6325000002253052],
[0.11490643398832881, -0.13191307112620382, 0.6330000002269912],
[0.11996931521875684, -0.1310639295464499, 0.6335000002369334],
[0.1255448010794605, -0.13051351751747672, 0.634500000241703],
[0.131235039123369, -0.12972142333131642, 0.6345000002429502],
[0.13564952775801156, -0.1284626575962195, 0.6355000002390202],
[0.14002894981096603, -0.12747311257612814, 0.6355000002390202],
[0.14509869978551576, -0.12650243629390218, 0.6360000002521434],
[0.15062965852221, -0.1254093120367765, 0.6360000002759644],
[0.15569952281668167, -0.12400974137714836, 0.6375000002758673],
[0.16136500546713747, -0.12227373321803935, 0.639000000284428],
[0.16723518352920896, -0.12029169477747732, 0.6385000002965223],
[0.17159944649125253, -0.11763461133260211, 0.6385000002965223],
[0.17661301960375322, -0.11530805628173656, 0.6380000002891614],
[0.1830679462794666, -0.11380310502041618, 0.6380000002874467],
[0.1894088087703991, -0.11171829903107866, 0.6365000002812095],
[0.19485757495556083, -0.10977027059275485, 0.634500000279207],
[0.20094566835123448, -0.108263188308559, 0.6335000002887905],
[0.20774451313962566, -0.10622957254291682, 0.633000000291941],
[0.21419489542440698, -0.10397876754353828, 0.6330000002886732],
[0.22001765287412348, -0.10177565175971255, 0.6320000002893353],
[0.22459418463756395, -0.0991975554527794, 0.6310000002899181],
[0.22931128888916966, -0.09614613815138677, 0.6300000002892464],
[0.2350212693554646, -0.09357679429750418, 0.6285000002794997],
[0.2400447809179774, -0.09111465715813691, 0.6275000002793709],
[0.24365920826822185, -0.08841775225684587, 0.6255000002897657],
[0.24783992879186015, -0.08596886191022964, 0.6230000002930033],
[0.25209132811634893, -0.08371414982636244, 0.6220000002941836],
[0.2549120415071027, -0.08197822472472026, 0.6215000003007634],
[0.2583453909844056, -0.08027065031598202, 0.6205000003024088],
[0.2612124081584077, -0.078036155852903, 0.6195000003051314],
[0.2634157384529241, -0.07623982058670686, 0.6190000003069873],
[0.2655012129155325, -0.07417700635316643, 0.6180000003128301],
[0.26684484181507745, -0.0716317231926783, 0.6165000003261748],
[0.2683843859420598, -0.06944555641010028, 0.616000000323771],
[0.26944714163286504, -0.06732475644492195, 0.615500000317703],
[0.2696256359479203, -0.06484283241691277, 0.6140000003225045],
[0.2698081889562699, -0.061929995976832825, 0.6135000003207657],
[0.2707747987267681, -0.0590717659167496, 0.6130000003257581],
[0.27141507681065274, -0.055754972011688095, 0.6130000003225817],
[0.27141507681065274, -0.0521550005562395, 0.6120000003203317],
[0.27162697230639266, -0.048921517033494115, 0.6115000003201492],
[0.2712165449592444, -0.04556433339614559, 0.6115000003201492],
[0.2710533579710559, -0.04225199481613459, 0.6125000003189762],
[0.26920027287528453, -0.038247888551232595, 0.6130000003167692],
[0.2655298776008298, -0.033841067355591455, 0.6140000003162056],
[0.26010088674500104, -0.03021001034876774, 0.6155000003176185],
[0.25266068721190593, -0.02699513201458546, 0.6180000003136404],
[0.24526515627680356, -0.02462432086315216, 0.6210000002948475],
[0.2388239390997816, -0.02347148352150792, 0.6230000002784465],
[0.23186428545224846, -0.02222696033348781, 0.6245000002677762],
[0.22462587496318082, -0.02027551867586198, 0.6245000002677762],
[0.21754194090848678, -0.018335313851449406, 0.6230000002621161],
[0.2093904341440251, -0.015995750858277723, 0.6220000002508159],
[0.2007062150110757, -0.014267414046184585, 0.6205000002550918],
[0.189643988375374, -0.014267414046184585, 0.6180000002588288],
[0.17661366840836434, -0.014605442723834718, 0.6180000002487335],
[0.16517114499068583, -0.01604992425823388, 0.6180000002487335],
[0.15339407946872308, -0.017672968881393487, 0.619500000254608],
[0.140272343257714, -0.01939075559348496, 0.6215000002586949],
[0.12761013802134574, -0.02130817903348829, 0.6235000002533083],
[0.11514814541935942, -0.022905727415144787, 0.6250000002514744],
[0.10216019815704815, -0.02363882562261116, 0.627500000239745],
[0.08812763276131051, -0.024051350116598095, 0.6295000002288569],
[0.07298271956148578, -0.02455329860594301, 0.6295000002288569],
[0.05830685975067429, -0.0245811579927856, 0.6295000002109471],
[0.044567879078102494, -0.0245811579927856, 0.6290000001972805],
[0.03189822778887969, -0.023701164587421397, 0.6290000001972805],
[0.020106065672627082, -0.022493586446242546, 0.6290000002055622],
[0.008929749780214649, -0.021775453202303802, 0.6290000002030895],
[-0.0016752142341067383, -0.021485057935930207, 0.6300000002042632],
[-0.011311928320428773, -0.02098258451082641, 0.6290000002045167],
[-0.020511080127675548, -0.02042003195854365, 0.6295000001964134],
[-0.03007365572995999, -0.019698599527978543, 0.6305000001752575],
[-0.03884113042506611, -0.01902372659467826, 0.6295000001964134],
[-0.04725981367766727, -0.018485741544464248, 0.6305000001752575],
[-0.05506460198820713, -0.01777652144651121, 0.6310000001560151],
[-0.06243992056552962, -0.01744912402774027, 0.6310000001618994],
[-0.06933779030696383, -0.017125618153421725, 0.6315000001577775],
[-0.07547975153943315, -0.01662304816791296, 0.6325000001476083],
[-0.08136574296321034, -0.01588977680012292, 0.6335000001470643],
[-0.08642906119530305, -0.015308588623057834, 0.6335000001470643],
[-0.09056348086994646, -0.015100656946485038, 0.6335000001415597],
[-0.0938309279189301, -0.014701279655922701, 0.6335000001424465],
[-0.09682083240898451, -0.014146385019036646, 0.6330000001476437],
[-0.09997756133305169, -0.013332854013036797, 0.633500000141335],
[-0.10311674235954948, -0.012481409106115388, 0.633500000141335],
[-0.10558731662868087, -0.011668056480342156, 0.6340000001353119],
[-0.10707886053980045, -0.010982113905313534, 0.6345000001353431],
[-0.10864000373741885, -0.010654337226353327, 0.6365000001367005],
[-0.11004422597102342, -0.010654337226353327, 0.6380000001390025],
[-0.11095397890610248, -0.010810275387699454, 0.6385000001379197],
[-0.11137903146579288, -0.011237077202193862, 0.6385000001379197],
[-0.1116671307113657, -0.011514194723625124, 0.640500000135857],
[-0.11229560676839641, -0.01185017041370256, 0.6420000001373987],
[-0.11285089579943892, -0.012469660835852442, 0.6430000001397561],
[-0.11337167822600816, -0.012931707624696483, 0.6455000001391468],
[-0.11381502215397153, -0.013242190643900743, 0.6475000001378387],
[-0.11428745258102818, -0.01382899022050025, 0.6490000001371397],
[-0.11469693369425525, -0.014326548959007316, 0.650000000137793],
[-0.11527876951407207, -0.014892836037263482, 0.6515000001387066],
[-0.11572568008652348, -0.01541942555421743, 0.6530000001391483],
[-0.11590082666863794, -0.01567946196168708, 0.6540000001346489],
[-0.11595948042343301, -0.015773516357402644, 0.6565000001294218],
[-0.11615621284661666, -0.015773516357402644, 0.6585000001280232],
[-0.11687836603911848, -0.015825224380114297, 0.660000000129559],
[-0.11741559898892673, -0.015825224380114297, 0.6615000001286755],
[-0.11761896759887114, -0.015981798229840655, 0.6625000001243716],
[-0.1178064223566116, -0.015981798229840655, 0.6635000001105725],
[-0.1178064223566116, -0.01554342208012197, 0.6645000001019068],
[-0.11792325875588072, -0.014652148713176622, 0.6655000001037817],
[-0.11815303635560968, -0.013619664421913777, 0.6645000001019068],
[-0.1179789983891266, -0.012718254705175763, 0.6640000001096265],
[-0.1179789983891266, -0.012243569193297086, 0.6625000001102609],
[-0.11747380788914677, -0.011451660729740293, 0.6625000001102609],
[-0.11705330854772787, -0.010333221072526776, 0.661000000105136],
[-0.11578166879949768, -0.009291891930385937, 0.66100000009541],
[-0.1143625382036468, -0.008280833606674535, 0.6620000000869408],
[-0.11385622294963677, -0.007297238775397253, 0.6620000000869408],
[-0.11374109265758273, -0.006561080961963007, 0.6630000000904394],
[-0.1134256055063096, -0.0060864221243838515, 0.6650000000932764],
[-0.11350762882008004, -0.0057642534929813714, 0.6660000000925981],
[-0.11392714160162634, -0.0057642534929813714, 0.6665000000948719],
[-0.11464507242343286, -0.006038029785019299, 0.6675000000989758],
[-0.11634425381934924, -0.006713682900730214, 0.6670000001036037],
[-0.11790440227930177, -0.007536930762197147, 0.6685000001000833],
[-0.1198343841665596, -0.008445373176731504, 0.6680000001013648],
[-0.12155182331250777, -0.009527241516919477, 0.6685000000962612],
[-0.1228670856074724, -0.01015136838701704, 0.6685000000962612],
[-0.12434377657802483, -0.01015136838701704, 0.6685000000962612],
[-0.12515820731101843, -0.010079558619543853, 0.6695000000835726],
[-0.12531023156097912, -0.009485042103675035, 0.669000000079881],
[-0.1260970175328325, -0.008220184115510403, 0.6705000000756316],
[-0.12665934249155814, -0.006660861052314094, 0.6715000000755328],
[-0.1268932565973544, -0.005116734856835232, 0.6720000000769842],
[-0.12761643771876663, -0.0033440232800579306, 0.6720000000769842],
[-0.12885202132584248, -0.0014305473529965, 0.6720000000749193],
[-0.13121750420216852, 0.0007490959753373518, 0.6720000000749193],
[-0.13544762015755868, 0.003204920294266045, 0.673500000071524],
[-0.14132921504516538, 0.007190489005380572, 0.6750000000145804],
    ]

    hand_poses = []
    for pose in hand_poses_raw:
        hand_point_trans = PoseStamped()
        hand_point_trans.pose.position.x = pose[0]
        hand_point_trans.pose.position.y = pose[1]
        hand_point_trans.pose.position.z = pose[2]
        hand_point_trans.pose.orientation.x = 0
        hand_point_trans.pose.orientation.y = 1.0
        hand_point_trans.pose.orientation.z = 0
        hand_point_trans.pose.orientation.w = 0

        hand_rel_base = do_transform_pose(hand_point_trans, ar_trans)
        print("Hand Point Rel. Base")
        print(hand_rel_base)
        hand_poses.append(hand_rel_base.pose)
    
    '''target_poses = [
        [0.985, 0.207, 0.443, 0.5, 0.5, 0.5, 0.5],
        [0.985, 0.18, 0.403, 0.5, 0.5, 0.5, 0.5],
        [0.935, 0.18, 0.443, 0.5, 0.5, 0.5, 0.5],
        [0.935, 0.207, 0.443, 0.5, 0.5, 0.5, 0.5],
    ]'''

    # pos = [hand_rel_base.pose.position.x, hand_rel_base.pose.position.y, hand_rel_base.pose.position.z, 0, 1, 0, 0]
    count = 0
    while not rospy.is_shutdown():
        # input('Press [ Enter ]: ')
        
        # Construct the request
        request = GetPositionIKRequest()

        request.ik_request.group_name = "right_arm"

        # If a Sawyer does not have a gripper, replace '_gripper_tip' with '_wrist' instead
        #link = "stp_022312TP99620_tip_1" # for amir

        link = "right_gripper_tip"

        request.ik_request.ik_link_name = link
        # request.ik_request.attempts = 20
        request.ik_request.pose_stamped.header.frame_id = "base"
        

        # Set the desired orientation for the end effector HERE

        # request.ik_request.pose_stamped.pose.position.x = pos[0]
        # request.ik_request.pose_stamped.pose.position.y = pos[1]
        # request.ik_request.pose_stamped.pose.position.z = pos[2] 
        # request.ik_request.pose_stamped.pose.orientation.x = pos[3]
        # request.ik_request.pose_stamped.pose.orientation.y = pos[4]
        # request.ik_request.pose_stamped.pose.orientation.z = pos[5]
        # request.ik_request.pose_stamped.pose.orientation.w = pos[6]
        
        try:
            # Send the request to the service
            # response = compute_ik(request)
            
            # # Print the response HERE
            # print("hi: ", response)
            group = MoveGroupCommander("right_arm")
            group.set_planner_id("RRTConnectkConfigDefault")
            group.set_goal_orientation_tolerance(100)
            group.set_goal_position_tolerance(0.05)
            
            # Setting position and orientation target
            #print(hand_poses[0])
            #group.set_position_target([hand_poses[0].position.x, hand_poses[0].position.y, hand_poses[0].position.z ])
            # group.set_pose_target(request.ik_request.pose_stamped)
            group.set_pose_targets(hand_poses)

            # TRY THIS
            # Setting just the position without specifying the orientation
            # group.set_position_target([0.5, 0.5, 0.0])

            # Plan IK
            plan = group.plan()
            
            #user_input = input("Enter 'y' if the trajectory looks safe on RVIZ")

            '''# Execute IK if safe
            if user_input == 'y':
                 group.execute(plan[1])
            elif user_input == 'n':
                sys.exit()'''
            group.execute(plan[1])
            count += 1
            if count % 20 == 0:
                user_input = input("Enter 'y' to exit")
                if user_input == 'y':
                    sys.exit()

        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

        

# Python's syntax for a main() method
if __name__ == '__main__':
    rospy.init_node('tagpos')
    #rate = rospy.Rate(0.01)
    main()
