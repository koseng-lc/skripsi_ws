/*
 * Multiple Object with Different Transformation
 * can be done with different model matrix and one shader program
 * create uniform array matrix
 */

#include "gait_tuner/viz.h"

JointStates& jstates_ref = JointStates::getInstance();

using namespace arma;

Viz::Viz():robot_object_(new RobotObject){
//    : viz_states_sub_(nh_.subscribe("/viz/states", 1, &Viz::vizStatesCb, this)){
    for(const auto id:JointData::ALL_ID){
        joint_pos_[id] = robot_object_->init_joint_pos[id];
        joint_axis_[id] = robot_object_->init_rotary_axis[id];
    }
}

Viz::~Viz(){
}

void Viz::initializeGL(){
    initializeOpenGLFunctions();

    glClearColor(0.0, 0.0, 0.0, 1.0);
    glEnable(GL_DEPTH_TEST);

    vertices_count_ = robot_object_->vertexCount();
    {
        std::stringstream vertex_path;
        std::stringstream fragment_path;
        vertex_path << ros::package::getPath("utils") << "/gui/simple.vert";
        fragment_path << ros::package::getPath("utils") << "/gui/simple.frag";

        shader_ = new QOpenGLShaderProgram;
        shader_->addShaderFromSourceFile(QOpenGLShader::Vertex, vertex_path.str().c_str());
        shader_->addShaderFromSourceFile(QOpenGLShader::Fragment, fragment_path.str().c_str());
        shader_->bindAttributeLocation("vertex", 0);
        shader_->bindAttributeLocation("normal", 1);
        shader_->bindAttributeLocation("instance", 2);
        shader_->link();
        shader_->bind();

        m_proj_loc_ = shader_->uniformLocation("m_proj");
        m_view_loc_ = shader_->uniformLocation("m_view");
        m_model_loc_ = shader_->uniformLocation("m_model");
        m_normal_loc_ = shader_->uniformLocation("m_normal");
        light_pos_loc_ = shader_->uniformLocation("light_pos");

        vao_.create();
        vao_.bind();

        buffer_.create();
        buffer_.bind();
        buffer_.allocate(robot_object_->robotData(), robot_object_->totalData() * sizeof(GLfloat));

        QOpenGLFunctions* f = QOpenGLContext::currentContext()->functions();
        f->glEnableVertexAttribArray(0); //enable vertex attribute for position
        f->glEnableVertexAttribArray(1); //enable vertex attribute for normal vector
        // unique ID in GPU, tuple size, data type, normalization, stride, start offset
        f->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat), 0);
        f->glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat), reinterpret_cast<void*>(3*sizeof(GLfloat)));

        buffer_.release();

        instance_buffer_.create();
        instance_buffer_.bind();
        instance_buffer_.allocate(robot_object_->instData(), robot_object_->instCount() * sizeof(GLfloat));

        f->glEnableVertexAttribArray(2);
        // only floating point for scalar value, cast in GLSL
        f->glVertexAttribPointer(2, 1, GL_FLOAT, GL_FALSE, 1*sizeof(GLfloat), 0);

        instance_buffer_.release();

        vao_.release();

        shader_->setUniformValue(light_pos_loc_, QVector3D(.0, .0, -50.f));

        shader_->release();

    }

}

int angle_view_z = 0;
int angle_view_y = 0;
int angle_view_x = 0;
float radius = 90.0f;
void Viz::paintGL(){
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    m_view_.setToIdentity();
    //polar to cartesian
    auto pos_x = -radius*cos(angle_view_x*MathUtils::DEG2RAD)*sin(angle_view_y*MathUtils::DEG2RAD);
    auto pos_y = radius*sin(angle_view_x*MathUtils::DEG2RAD);
    auto pos_z = radius*cos(angle_view_x*MathUtils::DEG2RAD)*cos(angle_view_y*MathUtils::DEG2RAD);
    QVector3D eye_pos(0,0,0);
    m_view_.lookAt(QVector3D(eye_pos.x() + pos_x, eye_pos.y() + pos_y, eye_pos.z() + pos_z),
                   QVector3D(.0, -5.0, .0),
                   QVector3D(.0, 1.0, .0));

    QMatrix3x3 m_normal[JointData::NUM_OF_JOINTS]; // to stay normal vectors are normal
    for(int i(0);i<JointData::NUM_OF_JOINTS;i++){
//        qDebug() << m_model_[i];
        m_normal[i] = m_model_[i].normalMatrix();
    }
    //Transform the model here is valid !!!!

    vao_.bind();
    shader_->bind();
    shader_->setUniformValue(m_proj_loc_, m_proj_);
    shader_->setUniformValue(m_view_loc_, m_view_);
    shader_->setUniformValueArray(m_model_loc_, m_model_, JointData::NUM_OF_JOINTS);
    // transform normal vectors are different with common vectors, because it should remain normal vectors
    // so it needs special matrix transformation
    shader_->setUniformValueArray(m_normal_loc_, m_normal, JointData::NUM_OF_JOINTS);

    glDrawArrays(GL_TRIANGLES, 0, vertices_count_);
    shader_->release();
    vao_.release();
}

void Viz::resizeGL(int w, int h){
    m_proj_.setToIdentity();
    m_proj_.perspective(45.0f, (float)w/h, .01f, 150.0f);
}

void Viz::keyPressEvent(QKeyEvent *e){
    (void)e;
//    if(e->key() == Qt::Key_A){
//        updateAngle(JointData::R_HIP_YAW, JointStates::getInstance().getAngle(JointData::R_HIP_YAW));
//        update();
//    }else if(e->key() == Qt::Key_S){
//        updateAngle(JointData::R_HIP_ROLL, 8);
//        update();
//    }else if(e->key() == Qt::Key_D){
//        updateAngle(JointData::R_KNEE, 8);
//        update();
//    }else if(e->key() == Qt::Key_W){
//        updateAngle(JointData::R_ANK_PITCH, 8);
//        update();
//    }else if(e->key() == Qt::Key_Q){
//        updateAngle(JointData::R_ANK_ROLL, 8);
//        update();
//    }else if(e->key() == Qt::Key_Z){
//        updateAngle(JointData::R_SHO_PITCH, 8);
//        update();
//    }else if(e->key() == Qt::Key_X){
//        updateAngle(JointData::R_SHO_ROLL, 8);
//        update();
//    }else if(e->key() == Qt::Key_C){
//        updateAngle(JointData::R_ELBOW, 8);
//        update();
//    }
}

void Viz::mousePressEvent(QMouseEvent *e){
    mouse_ref_ = e->pos();
}

void Viz::mouseMoveEvent(QMouseEvent *e){
    angle_view_y += (e->pos().x() - mouse_ref_.x());
    angle_view_x += (e->pos().y() - mouse_ref_.y());
    if(angle_view_x > 360.0)angle_view_x = .0;
    else if(angle_view_x < -360.0)angle_view_x = .0;

    if(angle_view_y > 360.0)angle_view_y = .0;
    else if(angle_view_y < -360.0)angle_view_y = .0;

    mouse_ref_ = e->pos();

//    qDebug() << angle_view_x << " ; " << angle_view_y;

    update();
}

void Viz::updateAngle(JointData::LinkID _joint_id, double _diff_angle){
    if( _joint_id != JointData::NONE){

        auto current_pivot = joint_pos_[_joint_id];
        auto current_rot_axis = joint_axis_[_joint_id];

        current_tf_.setToIdentity();
        current_tf_.translate(current_pivot);
        current_tf_.rotate(_diff_angle, current_rot_axis);
        current_tf_.translate(-current_pivot);

        m_model_[_joint_id] = current_tf_ * m_model_[_joint_id];

        current_rot_.setToIdentity();
        current_rot_.rotate(_diff_angle, current_rot_axis);
        joint_axis_rot_[_joint_id] = current_rot_ * joint_axis_rot_[_joint_id];

        JointData::LinkID child_id = jstates_ref.getChild(_joint_id);
        updateModel(child_id);
        updateJointPos(child_id);
        updateRotaryAxis(child_id);
    }
}         

void Viz::updateModel(JointData::LinkID _joint_id){
    if( _joint_id != JointData::NONE){
        m_model_[_joint_id] = current_tf_ * m_model_[_joint_id];
        updateModel(jstates_ref.getChild(_joint_id));
    }
}

void Viz::updateAngle(){
    for(auto id:JointData::ALL_ID){
        auto init_pos = RobotObject::qt2arma(robot_object_->init_joint_pos[id]);
        auto curr_pos = jstates_ref.getPos(id);
        auto position = curr_pos - init_pos;
        auto rot = RobotObject::arma2qt(jstates_ref.getRotMat(id));
        auto rot_data = rot.data();
//        m_model_[id] = QMatrix4x4(rot_data[0],rot_data[1],rot_data[2],position.at(1,0),
//                                  rot_data[3],rot_data[4],rot_data[5],position.at(2,0),
//                                  rot_data[6],rot_data[7],rot_data[8],position.at(0,0),
//                                           .0,         .0,         .0,            1.0);
        m_model_[id] = QMatrix4x4(rot_data[0],rot_data[1],rot_data[2], .0,
                                  rot_data[3],rot_data[4],rot_data[5], .0,
                                  rot_data[6],rot_data[7],rot_data[8], .0,
                                           .0,         .0,         .0,1.0);
    }
}

void Viz::updateJointPos(JointData::LinkID _joint_id){
    if(_joint_id != JointData::NONE){
        joint_pos_[_joint_id] = QVector4D(m_model_[_joint_id] * QVector4D(robot_object_->init_joint_pos[_joint_id], 1.0)).toVector3D();
        updateJointPos(jstates_ref.getChild(_joint_id));
    }
}

void Viz::updateRotaryAxis(JointData::LinkID _joint_id){
    if(_joint_id != JointData::NONE){
        joint_axis_rot_[_joint_id] = current_rot_ * joint_axis_rot_[_joint_id];
        joint_axis_[_joint_id] = QVector4D(joint_axis_rot_[_joint_id] * QVector4D(robot_object_->init_rotary_axis[_joint_id], 1.0)).toVector3D();
        updateRotaryAxis(jstates_ref.getChild(_joint_id));
    }
}

void Viz::actionForState(JointData::LinkID _joint_id, double _delta_theta){
    updateAngle(_joint_id, _delta_theta);
}

//Debugging Only
//QMatrix4x4 a(1, 0, 0, 0,
//             0, 0.913545, -0.406737, -0.122265,
//             0, 0.406737, 0.913545, 0.575212,
//             0, 0, 0, 1);

//QMatrix4x4 b(1, 0, 0, 0,
//             0, 0.990268, -0.139173, -0.930727,
//             0, 0.139173, 0.990268, 1.92138,
//             0, 0, 0, 1);
