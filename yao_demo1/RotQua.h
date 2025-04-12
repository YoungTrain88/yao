#ifndef _ROTQUA_H
#define _ROTQUA_H
/**
 * @brief This class is used to represent rotation in 3d space.
 * */
class Rotation {
public:
  Rotation();
  /**
   * @brief Construct
   * @param[in] Xx \ref data[0]
   * @param[in] Yx \ref data[1]
   * @param[in] Zx \ref data[2]
   * @param[in] Xy \ref data[3]
   * @param[in] Yy \ref data[4]
   * @param[in] Zy \ref data[5]
   * @param[in] Xz \ref data[6]
   * @param[in] Yz \ref data[7]
   * @param[in] Zz \ref data[8]
  */
  //Rotation(double Xx, double Yx, double Zx, double Xy, double Yy, double Zy,
  //         double Xz, double Yz, double Zz);
  /**
   * @brief Convert quaternion to Rotation object.
   * q=[w,(x,y,z)], w is the scalar, (x,y,z)is the direction vector.
   * @param[in] x X of the quaternion
   * @param[in] y Y of the quaternion
   * @param[in] z Z of the quaternion
   * @param[in] w W of the quaternion
  */
  void quaternion(double x, double y, double z, double w);
  void display();
  /**
   * @brief Get quaternion from the Rotation.
   * @param[in] x Reference to quaternion x
   * @param[in] y Reference to quaternion y
   * @param[in] z Reference to quaternion z
   * @param[in] w Reference to quaternion w
  */
  //void getQuaternion(double &x, double &y, double &z, double &w) const;
  /**
   * @brief data of the rotation matrix
   * @note the rotation matrix is defined as:
   * [data[0], data[1], data[2];
   *  data[3], data[4], data[5];
   *  data[6], data[7], data[8]]
   * */
  double data[9];
};

#endif