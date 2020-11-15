%%Inizializzation of connection at ros node matlab
rosinit('127.0.0.1',11311);
tree = rostf
pause(1);

node = ros.Node('/tf_parse');
tftree = ros.TransformationTree(node);
pause(1);
frame=tree.AvailableFrames;
frame_tf=["" "" "" "" "" "" "" ];
frame_tf(1)=frame(1);
frame_tf(7)=frame(2);
for i=2:6
    frame_tf(i)=frame(i+1);
end
while true
    for i=1:6
    tf = getTransform(tftree,frame_tf(i+1),frame_tf(i));
    fprintf("\n%s\n","-----Transformation form "+frame_tf(i)+" to "+frame_tf(i+1)+"---------");
    fprintf("%s\n","-----Translation------")
    x_translation=tf.Transform.Translation.X;
    y_translation=tf.Transform.Translation.Y;
    z_translation=tf.Transform.Translation.Z;   
    fprintf("x:%f\n",x_translation);
    fprintf("y:%f\n",y_translation);
    fprintf("z:%f\n",z_translation);
    x_rot=tf.Transform.Rotation.X;
    y_rot=tf.Transform.Rotation.Y;
    z_rot=tf.Transform.Rotation.Z;
    w_rot=tf.Transform.Rotation.W;
    quat=quaternion(w_rot,x_rot,y_rot,z_rot);
    fprintf("-----Rotation Quaternion------\n");
    fprintf("X:%f\n",x_rot);
    fprintf("Y:%f\n",y_rot);
    fprintf("Z:%f\n",z_rot);
    fprintf("W:%f\n",w_rot);
    fprintf("%s\n","-----Axis/Angle----");
    axisangle=quat2axang(quat);
    fprintf("%f ",axisangle(1:3));
    fprintf("\ntheta:%f\n",axisangle(4))
    fprintf("%s\n","-----Rotation------")
    rotation_matrix=quat2rotm(quat)
    fprintf("%s\n","----Eulero angle RPY-----")
    RPY=quat2eul(quat);
    fprintf("%f ",RPY);
    pause(3)
    end
end
%for close ros note use rossshutdown