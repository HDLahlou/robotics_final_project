#include <gazebo/gazebo.hh>
#include <gazebo/rendering/rendering.hh>

namespace gazebo
{
  void SetCustomShadows(
    Ogre::SceneManager *manager,
    const Ogre::ShadowTechnique technique,
    const size_t directional,
    const size_t point,
    const size_t spotlight
  ) {
        size_t total = directional + point + spotlight;

        manager->setShadowTechnique(technique);
        manager->setShadowTextureCount(total);

        manager->setShadowTextureCountPerLightType(Ogre::Light::LT_DIRECTIONAL, directional);
        manager->setShadowTextureCountPerLightType(Ogre::Light::LT_POINT, point);
        manager->setShadowTextureCountPerLightType(Ogre::Light::LT_SPOTLIGHT, spotlight);

        for (unsigned int i = 0; i < total; i++)
        {
            manager->setShadowTextureConfig(i, 8096u, 8096u, Ogre::PF_FLOAT32_R);
        }

        manager->setShadowTextureSelfShadow(true);
  }

  void CreateSpotlight(
    Ogre::SceneManager *manager,
    const Ogre::String &name,
    const Ogre::ColourValue &diffuse,
    const Ogre::ColourValue &specular,
    const Ogre::Radian &inner_angle,
    const Ogre::Radian &outer_angle,
    const Ogre::Real falloff,
    const float range,
    const float attenuation,
    const Ogre::Vector3 &position,
    const Ogre::Vector3 &direction
  ) {
        Ogre::Light *light = manager->createLight(name);
        light->setType(Ogre::Light::LT_SPOTLIGHT);

        light->setDiffuseColour(diffuse);
        light->setSpecularColour(specular);
        light->setSpotlightRange(inner_angle, outer_angle, falloff);
        light->setAttenuation(range, attenuation, 0.0, 0.0);
        light->setCastShadows(true);

        Ogre::SceneNode *node = manager->getRootSceneNode()->createChildSceneNode();

        node->attachObject(light);
        node->setDirection(direction);
        node->setPosition(position);
  }

  class CustomShadows : public SystemPlugin
  {
    private: std::vector<event::ConnectionPtr> connections;
    private: bool initialized = false;

    public: CustomShadows() : SystemPlugin()
      {
        std::cout << "Custom Shadows Plugin: Loading...\n";
      }

    public: void OnPreRender()
    {
      if (this->initialized)
        return;

      rendering::ScenePtr scene = rendering::get_scene();

      if (!scene || !scene->Initialized())
        return;

      Ogre::SceneManager *manager = scene->OgreSceneManager();

      SetCustomShadows(
        manager,
        Ogre::SHADOWTYPE_STENCIL_MODULATIVE,
        3, 3, 3
      );

      CreateSpotlight(
        manager,
        "spotlight_1",
        Ogre::ColourValue(1.0, 1.0, 1.0),
        Ogre::ColourValue(1.0, 1.0, 1.0),
        Ogre::Radian(0.20), Ogre::Radian(0.50), 1.00,
        6.0, 0.80,
        Ogre::Vector3(1.8, 2.0, 0.25),
        Ogre::Vector3(1.0, 0.0, 0.0)
      );

      this->initialized = true;
    }

    public: void Load(int, char **)
    {
      this->connections.push_back(
        event::Events::ConnectPreRender(
          std::bind(&CustomShadows::OnPreRender, this)
        )
      );

      std::cout << "Custom Shadows Plugin: Loaded!\n";
    }
  };

GZ_REGISTER_SYSTEM_PLUGIN(CustomShadows)

}
