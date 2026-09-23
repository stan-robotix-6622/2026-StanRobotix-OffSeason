#pragma once

#include <memory>
#include <string_view>

#include <networktables/DoubleTopic.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>

namespace stan {

struct PIDGains {
  double kP{0.0};
  double kI{0.0};
  double kD{0.0};
  double kS{0.0};
  double kV{0.0};
  double kG{0.0};

  bool operator==(const PIDGains& iOther) const = default;
};

enum class TunablePreset {
  kFlywheel,
  kPivot,
  kRoller,
  kSwerveDrive,
  kSwerveSteer
};

class StanTunablePID {
 public:
  StanTunablePID(std::string_view iTableName, const PIDGains& iDefaultGains)
      : mCurrentGains{iDefaultGains} {
    mTable = nt::NetworkTableInstance::GetDefault().GetTable(iTableName);

    auto topicP = mTable->GetDoubleTopic("kP");
    auto topicI = mTable->GetDoubleTopic("kI");
    auto topicD = mTable->GetDoubleTopic("kD");
    auto topicS = mTable->GetDoubleTopic("kS");
    auto topicV = mTable->GetDoubleTopic("kV");
    auto topicG = mTable->GetDoubleTopic("kG");

    mPubP = topicP.Publish();
    mPubI = topicI.Publish();
    mPubD = topicD.Publish();
    mPubS = topicS.Publish();
    mPubV = topicV.Publish();
    mPubG = topicG.Publish();

    mPubP.SetDefault(iDefaultGains.kP);
    mPubI.SetDefault(iDefaultGains.kI);
    mPubD.SetDefault(iDefaultGains.kD);
    mPubS.SetDefault(iDefaultGains.kS);
    mPubV.SetDefault(iDefaultGains.kV);
    mPubG.SetDefault(iDefaultGains.kG);

    mSubP = topicP.Subscribe(iDefaultGains.kP);
    mSubI = topicI.Subscribe(iDefaultGains.kI);
    mSubD = topicD.Subscribe(iDefaultGains.kD);
    mSubS = topicS.Subscribe(iDefaultGains.kS);
    mSubV = topicV.Subscribe(iDefaultGains.kV);
    mSubG = topicG.Subscribe(iDefaultGains.kG);
  }

  StanTunablePID(std::string_view iTableName, TunablePreset iPreset)
      : StanTunablePID(iTableName, getPreset(iPreset)) {}

  bool hasChanged() {
    PIDGains latestGains{
        mSubP.Get(mCurrentGains.kP),
        mSubI.Get(mCurrentGains.kI),
        mSubD.Get(mCurrentGains.kD),
        mSubS.Get(mCurrentGains.kS),
        mSubV.Get(mCurrentGains.kV),
        mSubG.Get(mCurrentGains.kG)};

    if (latestGains != mCurrentGains) {
      mCurrentGains = latestGains;
      return true;
    }
    return false;
  }

  PIDGains getGains() const {
    return PIDGains{
        mSubP.Get(mCurrentGains.kP),
        mSubI.Get(mCurrentGains.kI),
        mSubD.Get(mCurrentGains.kD),
        mSubS.Get(mCurrentGains.kS),
        mSubV.Get(mCurrentGains.kV),
        mSubG.Get(mCurrentGains.kG)};
  }

  void publishCurrentGains(const PIDGains& iGains) {
    mPubP.Set(iGains.kP);
    mPubI.Set(iGains.kI);
    mPubD.Set(iGains.kD);
    mPubS.Set(iGains.kS);
    mPubV.Set(iGains.kV);
    mPubG.Set(iGains.kG);
    mCurrentGains = iGains;
  }

  void publishCurrentGains() {
    publishCurrentGains(mCurrentGains);
  }

  static PIDGains getPreset(TunablePreset iPreset) {
    switch (iPreset) {
      case TunablePreset::kFlywheel:
        return PIDGains{0.1, 0.0, 0.0, 0.05, 0.12, 0.0};
      case TunablePreset::kPivot:
        return PIDGains{40.0, 0.0, 0.5, 0.0, 0.0, 0.3};
      case TunablePreset::kRoller:
        return PIDGains{0.05, 0.0, 0.0, 0.05, 0.12, 0.0};
      case TunablePreset::kSwerveDrive:
        return PIDGains{0.1, 0.0, 0.0, 0.1, 0.12, 0.0};
      case TunablePreset::kSwerveSteer:
        return PIDGains{40.0, 0.0, 0.5, 0.0, 0.0, 0.0};
    }
    return PIDGains{};
  }

  std::shared_ptr<nt::NetworkTable> getTable() const {
    return mTable;
  }

 private:
  std::shared_ptr<nt::NetworkTable> mTable;
  PIDGains mCurrentGains;

  nt::DoubleSubscriber mSubP;
  nt::DoubleSubscriber mSubI;
  nt::DoubleSubscriber mSubD;
  nt::DoubleSubscriber mSubS;
  nt::DoubleSubscriber mSubV;
  nt::DoubleSubscriber mSubG;

  nt::DoublePublisher mPubP;
  nt::DoublePublisher mPubI;
  nt::DoublePublisher mPubD;
  nt::DoublePublisher mPubS;
  nt::DoublePublisher mPubV;
  nt::DoublePublisher mPubG;
};

} // namespace stan
