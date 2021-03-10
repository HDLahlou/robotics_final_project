#include <gazebo/gazebo.hh>
#include <gazebo/rendering/rendering.hh>
#include <ignition/math.hh>

namespace gazebo
{
  class FixLights : public SystemPlugin
  {
    public: FixLights() : SystemPlugin()
      {
        std::cout << "Helllllooooooooooooooooooooooooo 1111111111\n";
      }

    public: void Update()
    {
      if (this->initialized)
        return;

      rendering::ScenePtr scene = rendering::get_scene();

      if (!scene || !scene->Initialized())
        return;

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
      Ogre::Light *pointLight1 = sceneManager->createLight("spot_green");
      pointLight1->setDiffuseColour(0, 1, 0);
      pointLight1->setSpecularColour(0, 1, 0);
      pointLight1->setType(Ogre::Light::LT_SPOTLIGHT);
      pointLight1->setCastShadows(true);
      Ogre::SceneNode *node1 = sceneManager->getRootSceneNode()->createChildSceneNode();
      node1->attachObject(pointLight1);
      node1->setDirection(1, 1, 0.2);
      node1->setPosition(gazebo::rendering::Conversions::Convert(ignition::math::Vector3d(5, 5, 3)));

      this->initialized = true;
    }

    public: void Load(int, char **)
    {
      std::cout << "loaddd\n";

      this->connections.push_back(
        event::Events::ConnectPreRender(
          std::bind(&FixLights::Update, this)));
    }

    private: std::vector<event::ConnectionPtr> connections;

    private: bool initialized = false;
  };
GZ_REGISTER_SYSTEM_PLUGIN(FixLights)
}
