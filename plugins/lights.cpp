#include <gazebo/gazebo.hh>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include "Ogre.h"

namespace gazebo
{
  class RoboticsFinalProject : public WorldPlugin
  {
    public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
    {
      gazebo::rendering::ScenePtr scene = gazebo::rendering::get_scene();
      Ogre::SceneManager *sceneManager = scene->OgreSceneManager();

      sceneManager->setShadowTechnique(Ogre::SHADOWTYPE_STENCIL_MODULATIVE); // This enables shadows, but pixelated lightings
      sceneManager->setShadowTextureCount(9);
      sceneManager->setShadowTextureCountPerLightType(Ogre::Light::LT_DIRECTIONAL, 3);
      sceneManager->setShadowTextureCountPerLightType(Ogre::Light::LT_POINT, 3);
      sceneManager->setShadowTextureCountPerLightType(Ogre::Light::LT_SPOTLIGHT, 3);
      for (unsigned int i = 0; i < 9; i++)
      {
          sceneManager->setShadowTextureConfig(i, 8096u, 8096u, Ogre::PF_FLOAT32_R);
      }
      sceneManager->setShadowTextureSelfShadow(true);

      // add green light
      Ogre::Light *pointLight1 = sceneManager->createLight("point_green");
      pointLight1->setDiffuseColour(0, 1, 0);
      pointLight1->setSpecularColour(0, 1, 0);
      pointLight1->setType(Ogre::Light::LT_POINT);
      pointLight1->setCastShadows(true);
      Ogre::SceneNode *node1 = sceneManager->getRootSceneNode()->createChildSceneNode();
      node1->attachObject(pointLight1);
      node1->setDirection(1, 1, 0.2);
      node1->setPosition(gazebo::rendering::Conversions::Convert(ignition::math::Vector3d(5, 5, 3)));

      // add red light
      Ogre::Light *pointLight2 = sceneManager->createLight("point_red");
      pointLight2->setDiffuseColour(1, 0, 0);
      pointLight2->setSpecularColour(1, 0, 0);
      pointLight2->setType(Ogre::Light::LT_POINT);
      pointLight2->setCastShadows(true);
      Ogre::SceneNode *node2 = sceneManager->getRootSceneNode()->createChildSceneNode();
      node2->attachObject(pointLight2);
      node2->setDirection(1, -1, 0.2);
      node2->setPosition(gazebo::rendering::Conversions::Convert(ignition::math::Vector3d(5, -5, 3)));
    }
  };
GZ_REGISTER_WORLD_PLUGIN(RoboticsFinalProject)
}
