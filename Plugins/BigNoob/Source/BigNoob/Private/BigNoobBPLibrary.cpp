// Copyright Epic Games, Inc. All Rights Reserved.

#include "BigNoobBPLibrary.h"
#include "BigNoob.h"

//-------------------------------------------------------------------------------------------------------------------

FVector CalculateCentroid(const TArray<FVector>& Points)
{
	FVector Centroid(0.f, 0.f, 0.f);
	for (const FVector& Point : Points)
	{
		Centroid += Point;
	}
	return Centroid / Points.Num();
}

void RemoveOutliers(TArray<FVector>& Points, const FVector& Centroid, float Threshold)
{
	float DistanceThresholdSquared = Threshold * Threshold;
	for (int32 i = Points.Num() - 1; i >= 0; i--)
	{
		if (FVector::DistSquared(Points[i], Centroid) > DistanceThresholdSquared)
		{
			Points.RemoveAt(i);
		}
	}
}

FPlane FitPlaneToPoints(const TArray<FVector>& Points)
{
	FVector Centroid = CalculateCentroid(Points);

	// Construct the covariance matrix
	FMatrix CovarianceMatrix = FMatrix::Identity;
	for (const FVector& Point : Points)
	{
		FVector RelativePoint = Point - Centroid;
		for (int32 i = 0; i < 3; ++i)
			for (int32 j = 0; j < 3; ++j)
				CovarianceMatrix.M[i][j] += RelativePoint[i] * RelativePoint[j];
	}

	// Find the eigenvalues and eigenvectors of the covariance matrix
	// UE4/UE5 does not have a built-in method for this, so we approximate
	// by assuming the smallest eigenvalue corresponds to the smallest row sum
	FVector PlaneNormal;
	float MinRowSum = MAX_flt;
	for (int32 i = 0; i < 3; ++i)
	{
		float RowSum = CovarianceMatrix.M[i][0] + CovarianceMatrix.M[i][1] + CovarianceMatrix.M[i][2];
		if (RowSum < MinRowSum)
		{
			MinRowSum = RowSum;
			PlaneNormal = FVector(CovarianceMatrix.M[i][0], CovarianceMatrix.M[i][1], CovarianceMatrix.M[i][2]);
		}
	}

	PlaneNormal.Normalize();

	return FPlane(Centroid, PlaneNormal);
}

bool ConstructPlaneFromPoints(const FVector& A, const FVector& B, const FVector& C, FPlane& OutPlane)
{
	FVector AB = B - A;
	FVector AC = C - A;
	FVector CrossProduct = FVector::CrossProduct(AB, AC);

	// ��������Ĳ�����Ƚӽ��㣬���������㹲��
	if (CrossProduct.SizeSquared() <= KINDA_SMALL_NUMBER)
	{
		return false; // ���ߣ��޷�����ƽ��
	}

	FVector Normal = CrossProduct.GetSafeNormal();
	OutPlane = FPlane(A, Normal);
	return true; // �ɹ�����ƽ��
}

FPlane FindMedianPlane(const TArray<FVector>& HitPoints)
{
	TArray<FPlane> ValidPlanes;
	int32 NumPoints = HitPoints.Num();

	// �������е�����������ƽ��
	for (int32 i = 0; i < NumPoints - 2; ++i)
	{
		for (int32 j = i + 1; j < NumPoints - 1; ++j)
		{
			for (int32 k = j + 1; k < NumPoints; ++k)
			{
				FPlane NewPlane;
				if (ConstructPlaneFromPoints(HitPoints[i], HitPoints[j], HitPoints[k], NewPlane))
				{
					ValidPlanes.Add(NewPlane);
				}
			}
		}
	}

	// ����ÿ���������������з��߼нǵ��ܺ�
	float SmallestAngleSum = MAX_flt;
	FPlane MedianPlane = ValidPlanes[0];
	for (int32 i = 0; i < ValidPlanes.Num(); ++i)
	{
		float AngleSum = 0.f;
		for (int32 j = 0; j < ValidPlanes.Num(); ++j)
		{
			if (i != j)
			{
				// �õ������֮��ĽǶ�
				float DotProduct = FVector::DotProduct(ValidPlanes[i].GetSafeNormal(), ValidPlanes[j].GetSafeNormal());
				// ��ֹ�򸡵㾫�����⵼�µ������[-1, 1]�ķ�Χ
				DotProduct = FMath::Clamp(DotProduct, -1.f, 1.f);
				AngleSum += acosf(DotProduct);
			}
		}

		// ������λ��ƽ��
		if (AngleSum < SmallestAngleSum)
		{
			SmallestAngleSum = AngleSum;
			MedianPlane = ValidPlanes[i];
		}
	}

	return MedianPlane;
}

FQuat FindQuatFromPlane(const TArray<FVector>& HitPoints)
{
	if (HitPoints.Num() < 3)
	{
		UE_LOG(LogTemp, Warning, TEXT("Not enough points to define a plane."));
		return FQuat::Identity;
	}

	FPlane BestPlane = FindMedianPlane(HitPoints);
	FVector PlaneNormal = BestPlane.GetSafeNormal();

	// �ӷ��ߵ���Ԫ����ת���߼����ֲ���
	FVector UpVector(0.0f, 0.0f, 1.0f);
	FVector RotationAxis = FVector::CrossProduct(UpVector, PlaneNormal).GetSafeNormal();
	float RotationAngle = FMath::Acos(FVector::DotProduct(UpVector, PlaneNormal));
	FQuat RotationQuat = FQuat(RotationAxis, RotationAngle);

	return RotationQuat;
}

//-------------------------------------------------------------------------------------------------------------------

UBigNoobBPLibrary::UBigNoobBPLibrary(const FObjectInitializer& ObjectInitializer)
: Super(ObjectInitializer)
{

}

float UBigNoobBPLibrary::BigNoobSampleFunction(float Param)
{
	return -1;
}

void UBigNoobBPLibrary::ActorSceneComponentsAlignCollision(AActor* InActor)
{
	if (InActor == nullptr) 
	{
		UE_LOG(LogTemp, Warning, TEXT("Get A nullptr InActor"));
		return;
	}

	auto Root = InActor->GetRootComponent();
	if (Root == nullptr)
	{
		UE_LOG(LogTemp, Warning, TEXT("Get A nullptr Root"));
		return;
	}

	TArray<USceneComponent*> Children;
	Root->GetChildrenComponents(false,Children);

	for (auto Com : Children)
	{
		UStaticMeshComponent* SmCom = Cast<UStaticMeshComponent>(Com);
		if (SmCom)
		{
			auto WorldTransform = SmCom->GetComponentTransform();
			auto Bounds = SmCom->CalcBounds(WorldTransform);
			//UE_LOG(LogTemp, Warning, TEXT("[Origin : %s] [Extend : %s]"),*Bounds.Origin.ToString(), *Bounds.BoxExtent.ToString());

			TArray<FVector> HitPoints;
			FVector Min = Bounds.GetBox().Min;
			FVector Max = Bounds.GetBox().Max;
			float Z = Min.Z;
			float StepSize = 50.0f;
			for (float x = Min.X; x < Max.X; x += StepSize)
			{
				for (float y = Min.Y; y < Max.Y; y += StepSize)
				{
					FVector Start = FVector(x, y, Z);
					FVector End = FVector(x, y, Z - 1000.0f); 
					FHitResult HitResult;
					bool bHit = InActor->GetWorld()->LineTraceSingleByChannel(
						HitResult,
						Start,
						End,
						ECC_Visibility,
						FCollisionQueryParams()
					);

					if (bHit)
					{
						DrawDebugLine(InActor->GetWorld(), Start, HitResult.ImpactPoint,FColor::Red, false, 5.0f, 0, 1.0f);
						UE_LOG(LogTemp, Warning, TEXT("Hit at Location: %s"), *HitResult.ImpactPoint.ToString());
						HitPoints.Add(HitResult.ImpactPoint);
					}
				}
			}

			FQuat BestRotation = FindQuatFromPlane(HitPoints);;
			WorldTransform.SetRotation(BestRotation);
			SmCom->SetWorldTransform(WorldTransform);
		}
	}

}

