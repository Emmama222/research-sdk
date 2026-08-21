from typing import Optional
from enum import Enum


def _optional_proto_value(message, name):
    """Return None when an optional protobuf field was not provided."""
    has_field = getattr(message, "HasField", None)
    if has_field is not None:
        try:
            if not has_field(name):
                return None
        except ValueError:
            pass
    return getattr(message, name, None)


class FieldShapeType(Enum):
    Undefined = 0
    CenterCircle = 1
    TopTouchLine = 2
    BottomTouchLine = 3
    LeftGoalLine = 4
    RightGoalLine = 5
    HalfwayLine = 6
    CenterLine = 7
    LeftPenaltyStretch = 8
    RightPenaltyStretch = 9
    LeftFieldLeftPenaltyStretch = 10
    LeftFieldRightPenaltyStretch = 11
    RightFieldLeftPenaltyStretch = 12
    RightFieldRightPenaltyStretch = 13


class Vector2f ():
    def __init__(self,x: float,y: float):
        self.x: float= x
        self.y: float= y 
    
    @classmethod
    def from_proto (cls,vector):
        return cls(x=float(vector.x), y=float(vector.y))
    
    def __repr__(self):
        return f"[{self.x=} , {self.y=}]"

class FieldLines():
    def __init__(self,name : str, p1 : Vector2f, p2 : Vector2f, 
                 thickness : float, type : FieldShapeType=FieldShapeType.Undefined):
        ## Name of this field marking.
        self.name : str = name
        ## Start point of the line segment.
        self.p1 : Vector2f = p1
        ## End point of the line segment.
        self.p2 : Vector2f = p2
        ## Thickness of the line segment.
        self.thickness : float = thickness
        ## The type of this shape (Optional)
        self.type : Optional[FieldShapeType] = type
    
    def __repr__(self):
        return f"\n{self.name=} \t {self.p1=} \t {self.p2=} \t {self.thickness=} \t {self.type=}"
    def __str__(self):
        return f"{self.type=} : {self.name=}"
    
    @classmethod
    def from_proto(cls,fl) -> object | None:
        if fl is not None:
            return cls(name=str(fl.name),
            p1=Vector2f.from_proto(fl.p1) ,
            p2=Vector2f.from_proto(fl.p2),
            thickness=float(fl.thickness),
            type=getattr(fl,"type",FieldShapeType.Undefined)
            )
        else :
            return None
            


class FieldArcs():
    def __init__(self,name : str, center : Vector2f, radius : float,
                 a1 : float ,a2 : float, thickness : float, type : FieldShapeType=FieldShapeType.CenterCircle ):
        ## Name of this field marking.
        self.name : str = name 
        ## Center point of the circular arc.
        self.center : Vector2f = center 
        ## Radius of the arc.
        self.radius : float = radius 
        ## Start angle in counter-clockwise order.
        self.a1 : float = a1 
        ## End angle in counter-clockwise order.
        self.a2 : float = a2 
        ## Thickness of the arc.
        self.thickness : float = thickness 
        ## The type of this shape (Optional)
        self.type : Optional[FieldShapeType] = type 
    
    def __repr__(self):
        return f"{self.name=} \t {self.center=} \t {self.radius=} \t {self.a1=} \t {self.a2=} \t {self.thickness=}\t {self.type=}"
        
    @classmethod
    def from_proto(cls,fa) -> object | None :
        if fa is not None:
            return cls(name = str(fa.name),
            center = Vector2f.from_proto(fa.center),
            radius = float(fa.radius),
            a1 = float(fa.a1) ,
            a2 = float(fa.a2),
            thickness = float(fa.thickness),
            type=getattr(fa,"type",FieldShapeType.CenterCircle)
            )
        else : 
            return None
class FieldSize():
    def __init__(self, field_length:int, field_width:int, goal_width:int, goal_depth:int, boundary_width:int,
                 field_lines:list[FieldLines], field_arcs:list[FieldArcs],
                 penalty_area_depth:int=None ,penalty_area_width:int=None):
        self.field_length : int= field_length
        self.field_width : int= field_width
        self.goal_width : int= goal_width
        self.goal_depth : int= goal_depth
        self.boundary_width : int= boundary_width
        self.field_lines : list[FieldLines]= field_lines
        self.field_arcs : list[FieldArcs]= field_arcs
        self.penalty_area_depth : Optional[int]= penalty_area_depth #(Optional)
        self.penalty_area_width : Optional[int]= penalty_area_width #(Optional)
    def __repr__ (self):
        return (f"{self.field_length=} \t {self.field_width=} \t {self.goal_width=} \t {self.goal_depth=} \t {self.boundary_width=} \t {self.penalty_area_width=} \t {self.penalty_area_depth=} \n"
                + f"{self.field_lines=} \n"
                + f"{self.field_arcs=} \n"
                
        )
    @classmethod
    def from_proto(cls,fs) -> object | None:
        if fs is not None:
            return cls(field_length=int(fs.field_length),
                field_width=int(fs.field_width),
                goal_width=int(fs.goal_width),
                goal_depth=int(fs.goal_depth),
                boundary_width=int(fs.boundary_width),
                field_lines=[FieldLines.from_proto(line) for line in fs.field_lines],
                field_arcs=[FieldArcs.from_proto(arc) for arc in fs.field_arcs],
                penalty_area_depth=_optional_proto_value(fs, "penalty_area_depth"),
                penalty_area_width=_optional_proto_value(fs, "penalty_area_width")
            )
        else: 
            return None
class CameraCalibration():
    def __init__(self, camera_id : int, focal_length : float, principal_point_x : float, principal_point_y : float,
        distortion : float, q0 : float, q1 : float, q2 : float, q3 : float, tx : float, ty : float, tz : float, 
        derived_camera_world_tx : float=None, derived_camera_world_ty : float=None, derived_camera_world_tz : float=None,
        pixel_image_width : int=None, pixel_image_height : int=None):
        self.camera_id : int= camera_id 
        self.focal_length : float= focal_length 
        self.principal_point_x : float= principal_point_x 
        self.principal_point_y : float= principal_point_y 
        self.distortion : float= distortion 
        self.q0 : float= q0 
        self.q1 : float= q1 
        self.q2 : float= q2 
        self.q3 : float= q3 
        self.tx : float= tx 
        self.ty : float= ty 
        self.tz : float= tz 
        self.derived_camera_world_tx : Optional[float]= derived_camera_world_tx 
        self.derived_camera_world_ty : Optional[float]= derived_camera_world_ty 
        self.derived_camera_world_tz : Optional[float]= derived_camera_world_tz 
        self.pixel_image_width : Optional[int]= pixel_image_width 
        self.pixel_image_height : Optional[int]= pixel_image_height 
        
    def __repr__(self):
        return (f"\n{self.principal_point_x=}, {self.principal_point_y=}, {self.distortion=}\n"
        + f"{self.q0=}, {self.q1=}, {self.q2=},{self.q3=}\n"
        + f"{self.tx=}, {self.ty=}, {self.tz=}\n"
        + f"{self.derived_camera_world_tx=}, {self.derived_camera_world_ty=}, {self.derived_camera_world_tz=}\n"
        + f"{self.pixel_image_width=}, {self.pixel_image_height=}\n"
        )
        
    
    @classmethod
    def from_proto(cls, cc) -> object | None:
        if cc is not None:
            return cls(
                camera_id=int(cc.camera_id),
                focal_length=float(cc.focal_length),
                principal_point_x=float(cc.principal_point_x),
                principal_point_y=float(cc.principal_point_y),
                distortion=float(cc.distortion),
                q0=float(cc.q0),
                q1=float(cc.q1),
                q2=float(cc.q2),
                q3=float(cc.q3),
                tx=float(cc.tx),
                ty=float(cc.ty),
                tz=float(cc.tz),
                derived_camera_world_tx=getattr(cc,"derived_camera_world_tx", None),
                derived_camera_world_ty=getattr(cc,"derived_camera_world_ty", None),
                derived_camera_world_tz=getattr(cc,"derived_camera_world_tz", None),
                pixel_image_width=getattr(cc, "pixel_image_width", None),
                pixel_image_height=getattr(cc, "pixel_image_height", None)
            )
        else : 
            return None
class BallModelStraightTwoPhase():
    def __init__(self, acc_slide : float, acc_roll : float, k_switch : float):
        self.acc_slide : float=acc_slide ## Ball sliding acceleration [m/s^2] (should be negative)
        self.acc_roll : float=acc_roll  ## Ball rolling acceleration [m/s^2] (should be negative)
        self.k_switch : float=k_switch  ## Fraction of the initial velocity where the ball starts to roll
    
    def __repr__(self):
        return f"\t {self.acc_slide=} \t {self.acc_roll=} \t {self.k_switch=}"
    
    @classmethod
    def from_proto(cls,s2p) -> object | None:
        if s2p is not None:
            return cls(
                acc_slide=float(s2p.acc_slide),
                acc_roll=float(s2p.acc_roll),
                k_switch=float(s2p.k_switch)
            )
        else:
            return None

class BallModelChipFixedLoss():
    def __init__(self,damping_xy_first_hop : float, damping_xy_other_hops : float, damping_z : float):
        self.damping_xy_first_hop : float = damping_xy_first_hop 
        self.damping_xy_other_hops : float = damping_xy_other_hops  ## Chip kick velocity damping factor in XY direction for all following hops
        self.damping_z : float = damping_z ## Chip kick velocity damping factor in Z direction for all hops
    def __repr__(self):
        return f"\t {self.damping_xy_first_hop=} \t {self.damping_xy_other_hops=} \t {self.damping_z=}\n"
    
    @classmethod
    def from_proto(cls,cfl) -> object | None:
        if cfl is not None:
            return cls(
            damping_xy_first_hop=float(cfl.damping_xy_first_hop),
            damping_xy_other_hops=float(cfl.damping_xy_other_hops),
            damping_z=float(cfl.damping_z)
            )
        else:
            return None
        
class GeometryModels():
    def __init__(self,straight_two_phase:BallModelStraightTwoPhase=None, chip_fixed_loss:BallModelChipFixedLoss=None):
        self.straight_two_phase:BallModelStraightTwoPhase=straight_two_phase
        self.chip_fixed_loss:BallModelChipFixedLoss=chip_fixed_loss
    
    def __repr__(self):
        return f"{self.straight_two_phase=} \n {self.chip_fixed_loss=}"
    
    @classmethod
    def from_proto(cls,gm) -> object | None:
        if gm is not None:
            return cls(
                straight_two_phase=BallModelStraightTwoPhase.from_proto(getattr(gm,"straight_two_phase",None)),
                chip_fixed_loss=BallModelChipFixedLoss.from_proto(getattr(gm,"chip_fixed_loss",None))
            )
        else:
            return None
        
class GeometryData():
    """
    Geometry data of world Model
    """
    def __init__(self, field:FieldSize,calibration:CameraCalibration,models:GeometryModels=None):
        self.field : FieldSize = field
        self.calibration : CameraCalibration = calibration
        self.models : GeometryModels= models
    
    def __repr__(self):
        return f"{self.field=} \n{self.calibration=} \n{self.models=}"
    
    @classmethod
    def from_proto(cls,gd):
        return cls(
            field=FieldSize.from_proto(gd.field),
            calibration=[CameraCalibration.from_proto(cc) for cc in gd.calib],
            models=GeometryModels.from_proto(getattr(gd,"models",None))
        )
        
    ## to do : need a merge function for grSim ? 
