/*
 * Copyright (c) 2011-2019, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include <osg/Billboard>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/LineWidth>
#include <osg/ShapeDrawable>
#include <osg/Texture2D>

#include "dart/gui/osg/ShapeFrameNode.hpp"
#include "dart/gui/osg/Utils.hpp"
#include "dart/gui/osg/render/PointCloudShapeNode.hpp"

#include "dart/dynamics/PointCloudShape.hpp"
#include "dart/dynamics/SimpleFrame.hpp"

namespace dart {
namespace gui {
namespace osg {
namespace render {

//==============================================================================
::osg::Drawable* createSquare(
    const ::osg::Vec3& corner,
    const ::osg::Vec3& width,
    const ::osg::Vec3& height,
    ::osg::ref_ptr<::osg::Image> image = nullptr)
{
  ::osg::ref_ptr<::osg::Geometry> geom = new ::osg::Geometry;

  ::osg::Vec3Array* coords = new ::osg::Vec3Array(4);
  (*coords)[0] = corner;
  (*coords)[1] = corner + width;
  (*coords)[2] = corner + width + height;
  (*coords)[3] = corner + height;

  geom->setVertexArray(coords);

  ::osg::Vec3Array* norms = new ::osg::Vec3Array(1);
  (*norms)[0] = width ^ height;
  (*norms)[0].normalize();

  geom->setNormalArray(norms, ::osg::Array::BIND_OVERALL);

  ::osg::Vec2Array* tcoords = new ::osg::Vec2Array(4);
  (*tcoords)[0].set(0.0f, 0.0f);
  (*tcoords)[1].set(1.0f, 0.0f);
  (*tcoords)[2].set(1.0f, 1.0f);
  (*tcoords)[3].set(0.0f, 1.0f);
  geom->setTexCoordArray(0, tcoords);

  geom->addPrimitiveSet(
      new ::osg::DrawArrays(::osg::PrimitiveSet::QUADS, 0, 4));

  if (image)
  {
    ::osg::StateSet* stateset = new ::osg::StateSet;
    ::osg::Texture2D* texture = new ::osg::Texture2D;
    texture->setImage(image);
    stateset->setTextureAttributeAndModes(
        0, texture, ::osg::StateAttribute::ON);
    geom->setStateSet(stateset);
  }

  return geom.release();
}

//==============================================================================
class QuadDrawable : public ::osg::Drawable
{
public:
  QuadDrawable()
  {
    // Do nothing
  }

protected:

private:
};

//==============================================================================
class PointShapeDrawable : public ::osg::ShapeDrawable
{
public:
  PointShapeDrawable()
  {
    // Do nothing
  }

  virtual void updateCenter(const Eigen::Vector3d& /*point*/)
  {
    // Do nothing
  }

  virtual void updateSize(double /*size*/)
  {
    // Do nothing
  }

  virtual void refresh(bool /*firstTime*/)
  {
  }

protected:
  virtual ~PointShapeDrawable()
  {
    // Do nothing
  }
};

//==============================================================================
class BoxDrawable final : public PointShapeDrawable
{
public:
  BoxDrawable(const Eigen::Vector3d& point, double size, const Eigen::Vector4d& color)
  {
    mShape = new ::osg::Box(eigToOsgVec3(point), static_cast<float>(size));
    setColor(eigToOsgVec4f(color));
    setShape(mShape);
  }

  void updateCenter(const Eigen::Vector3d& point) override
  {
    mShape->setCenter(eigToOsgVec3(point));

    dirtyBound();
    dirtyDisplayList();

    mUpdated = true;
  }

  void updateSize(double size) override
  {
    mShape->setHalfLengths(::osg::Vec3(
        static_cast<float>(size),
        static_cast<float>(size),
        static_cast<float>(size)));
    mUpdated = true;
  }

  void refresh(bool /*firstTime*/) override
  {
    if (mUpdated)
      setDataVariance(::osg::Object::DYNAMIC);
    else
      setDataVariance(::osg::Object::STATIC);

    mUpdated = false;
  }

protected:
  ~BoxDrawable() override
  {
    // Do nothing
  }

  ::osg::ref_ptr<::osg::Box> mShape;
  bool mUpdated{false};
};

//==============================================================================
class SphereDrawable final : public PointShapeDrawable
{
public:
  SphereDrawable(const Eigen::Vector3d& point, double size)
  {
    mShape = new ::osg::Sphere(eigToOsgVec3(point), static_cast<float>(size));
    setShape(mShape);
  }

  void updateCenter(const Eigen::Vector3d& point) override
  {
    mShape->setCenter(eigToOsgVec3(point));

    dirtyBound();
    dirtyDisplayList();

    mUpdated = true;
  }

  void updateSize(double size) override
  {
    mShape->setRadius(static_cast<float>(size));
    mUpdated = true;
  }

  void refresh(bool /*firstTime*/) override
  {
    if (mUpdated)
      setDataVariance(::osg::Object::DYNAMIC);
    else
      setDataVariance(::osg::Object::STATIC);

    mUpdated = false;
  }

protected:
  ~SphereDrawable() override
  {
    // Do nothing
  }

  ::osg::ref_ptr<::osg::Sphere> mShape;
  bool mUpdated{false};
};

//==============================================================================
class PointCloudShapeGeode : public ShapeNode, public ::osg::Group
//    public ::osg::Billboard
{
public:
  PointCloudShapeGeode(
      std::shared_ptr<dart::dynamics::PointCloudShape> shape,
      ShapeFrameNode* parent)
    : ShapeNode(shape, parent, this), mPointCloudShape(shape)
  {
    getOrCreateStateSet()->setMode(GL_BLEND, ::osg::StateAttribute::ON);

    extractData(true);
  }

  void refresh() override
  {
    mUtilized = true;

    extractData(false);
  }

  void extractData(bool /*firstTime*/)
  {
    // Create sub geode and add it as child
  }

  virtual void updateCenter(const Eigen::Vector3d& /*point*/)
  {
    // Do nothing
  }

  virtual void updateSize(double /*size*/)
  {
    // Do nothing
  }

  virtual void updateColor(const Eigen::Vector4d& /*color*/)
  {
    // Do nothing
  }

protected:
  ~PointCloudShapeGeode() override
  {
    // Do nothing
  }

  dart::dynamics::PointCloudShape::PointShapeType mPointShapeType{
      dart::dynamics::PointCloudShape::PointShapeType::BOX};

  std::shared_ptr<dart::dynamics::PointCloudShape> mPointCloudShape;
  std::vector<::osg::ref_ptr<PointShapeDrawable>> mDrawables;

private:
  ::osg::ref_ptr<PointShapeDrawable> createPointShapeDrawable(
      const Eigen::Vector3d& center, double size)
  {
    // TODO
    ::osg::ref_ptr<PointShapeDrawable> drawable
        = new SphereDrawable(center, size);

    return drawable;
  }
};

//==============================================================================
class PointGeode : public PointCloudShapeGeode
{
public:
protected:
private:
};

//==============================================================================
class BoxGeode final : public PointCloudShapeGeode
{
public:
  BoxGeode(const Eigen::Vector3d& point, double size, const Eigen::Vector4d& color,
           std::shared_ptr<dart::dynamics::PointCloudShape> shape,
           ShapeFrameNode* parent)
    : PointCloudShapeGeode(shape, parent)
  {
    mBoxDrawable = new BoxDrawable(point, size, color);
    addDrawable(mBoxDrawable);
  }

  void updateCenter(const Eigen::Vector3d& point) override
  {
    mBoxDrawable->updateCenter(point);
  }

  void updateSize(double size) override
  {
    mBoxDrawable->updateSize(size);
  }

  void updateColor(const Eigen::Vector4d& color) override
  {
    mBoxDrawable->setColor(eigToOsgVec4f(color));
  }

protected:
  ::osg::ref_ptr<BoxDrawable> mBoxDrawable;
private:
};

//==============================================================================
class PointCloudShapeBillboardGeode : public ShapeNode, public ::osg::Billboard
//    public ::osg::Billboard
{
public:
  PointCloudShapeBillboardGeode(
      std::shared_ptr<dart::dynamics::PointCloudShape> shape,
      ShapeFrameNode* parent)
    : ShapeNode(shape, parent, this), mPointCloudShape(shape)
  {
    getOrCreateStateSet()->setMode(GL_BLEND, ::osg::StateAttribute::ON);

    extractData(true);
  }

  void refresh() override
  {
    mUtilized = true;

    extractData(false);
  }

  void extractData(bool /*firstTime*/)
  {
  }

  virtual void updateCenter(const Eigen::Vector3d& /*point*/)
  {
    // Do nothing
  }

  virtual void updateSize(double /*size*/)
  {
    // Do nothing
  }

  virtual void updateColor(const Eigen::Vector4d& /*color*/)
  {
    // Do nothing
  }

protected:
  ~PointCloudShapeBillboardGeode() override
  {
    // Do nothing
  }

  std::shared_ptr<dart::dynamics::PointCloudShape> mPointCloudShape;
  ::osg::ref_ptr<PointShapeDrawable> mDrawable;

private:
};

//==============================================================================
class BillboardQuadGeode : public PointCloudShapeBillboardGeode
{
public:
protected:
private:
};

//==============================================================================
class BillboardCircleGeode : public PointCloudShapeBillboardGeode
{
public:
protected:
private:
};

//==============================================================================
PointCloudShapeNode::PointCloudShapeNode(
    std::shared_ptr<dart::dynamics::PointCloudShape> shape,
    ShapeFrameNode* parent)
  : ShapeNode(shape, parent, this),
    mPointCloudShape(shape),
    mGeode(nullptr),
    mPointCloudVersion(dynamics::INVALID_INDEX)
{
  mNode = this;
  extractData(true);
  setNodeMask(mVisualAspect->isHidden() ? 0x0u : ~0x0u);
}

//==============================================================================
void PointCloudShapeNode::refresh()
{
  mUtilized = true;

  setNodeMask(mVisualAspect->isHidden() ? 0x0u : ~0x0u);

  if (mShape->getDataVariance() == dart::dynamics::Shape::STATIC
      && mPointCloudVersion == mPointCloudShape->getVersion())
  {
    return;
  }

  extractData(false);

  mPointCloudVersion = mPointCloudShape->getVersion();
}

//==============================================================================
template <typename T>
void refereshGeodes(
    std::shared_ptr<dart::dynamics::PointCloudShape> mPointCloudShape,
    std::vector<::osg::ref_ptr<T>>& mGeodes,
    PointCloudShapeNode* node)
{
  const auto visualSize = mPointCloudShape->getVisualSize();
  const auto& points = mPointCloudShape->getPoints();
  const auto& colors = mPointCloudShape->getColors();

  // Pre-allocate for the case that the size of new points are greater than
  // previous update
  mGeodes.reserve(points.size());

  // Update position of cache boxes. The number of being updated boxes is
  // whichever the lower number of cache boxes and new points.
  const auto numUpdatingPoints = std::min(mGeodes.size(), points.size());
  for (auto i = 0u; i < numUpdatingPoints; ++i)
  {
    mGeodes[i]->updateCenter(points[i]);
    mGeodes[i]->updateColor(colors[i]);
    mGeodes[i]->refresh();
  }

  // If the number of new points is greater than cache box, then create new
  // boxes that many.
  for (auto i = mGeodes.size(); i < points.size(); ++i)
  {
    auto osgSphere = node->createGeode(points[i], visualSize, colors[i]);
    mGeodes.emplace_back(osgSphere);
    node->addChild(mGeodes.back());
    mGeodes.back()->refresh();
  }

  // Fit the size of cache box list to the new points. No effect new boxes are
  // added to the list.
  if (mGeodes.size() > points.size())
  {
    node->removeChildren(
        static_cast<unsigned int>(points.size()),
        static_cast<unsigned int>(mGeodes.size() - points.size()));
    mGeodes.resize(points.size());
  }
}

//==============================================================================
void PointCloudShapeNode::extractData(bool firstTime)
{
  if (firstTime)
  {
    mPointShapeType = mPointCloudShape->getPointShapeType();
  }
  else
  {
    if (mPointShapeType != mPointCloudShape->getPointShapeType())
    {
      mGeodes.clear();
    }
  }

  refereshGeodes(mPointCloudShape, mGeodes, this);
}

//==============================================================================
PointCloudShapeNode::~PointCloudShapeNode()
{
  // Do nothing
}

//==============================================================================
::osg::ref_ptr<PointCloudShapeGeode> PointCloudShapeNode::createGeode(
    const Eigen::Vector3d& point, double size, const Eigen::Vector4d& color)
{
  ::osg::ref_ptr<PointCloudShapeGeode> geode
      = new BoxGeode(point, size, color, mPointCloudShape, mParentShapeFrameNode);

  return geode;
}

} // namespace render
} // namespace osg
} // namespace gui
} // namespace dart
