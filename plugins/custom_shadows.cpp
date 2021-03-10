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

  void CreateLight(
    Ogre::SceneManager *manager,
    const Ogre::String &name,
    const Ogre::Light::LightTypes type,
    const Ogre::ColourValue &diffuse,
    const Ogre::ColourValue &specular,
    const Ogre::Vector3 &position,
    const Ogre::Vector3 &direction
  ) {
        Ogre::Light *light = manager->createLight(name);

        light->setDiffuseColour(diffuse);
        light->setSpecularColour(specular);

        light->setType(type);
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

      CreateLight(
        manager,
        "spotlight_1",
        Ogre::Light::LT_SPOTLIGHT,
        Ogre::ColourValue(1, 0, 0),
        Ogre::ColourValue(1, 0, 0),
        Ogre::Vector3(5, 5, 3),
        Ogre::Vector3(1, 1, 0.2)
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
