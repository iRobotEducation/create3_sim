import math
from typing import Union

from launch import LaunchContext, SomeSubstitutionsType, Substitution


class RotationalOffsetX(Substitution):
    def __init__(
            self,
            offset: float,
            yaw: SomeSubstitutionsType
    ) -> None:
        self.__offset = offset
        self.__yaw = yaw

    def perform(
            self,
            context: LaunchContext = None,
    ) -> str:
        yaw = float(self.__yaw.perform(context))
        return f'{self.__offset * math.cos(yaw)}'


class RotationalOffsetY(Substitution):
    def __init__(
            self,
            offset: float,
            yaw: SomeSubstitutionsType
    ) -> None:
        self.__offset = offset
        self.__yaw = yaw

    def perform(
            self,
            context: LaunchContext = None,
    ) -> str:
        yaw = float(self.__yaw.perform(context))
        return f'{self.__offset * math.sin(yaw)}'


class OffsetParser(Substitution):
    def __init__(
            self,
            number: SomeSubstitutionsType,
            offset: Union[float, Substitution],
    ) -> None:
        self.__number = number
        self.__offset = offset

    def perform(
            self,
            context: LaunchContext = None,
    ) -> str:
        number = float(self.__number.perform(context))
        if isinstance(self.__offset, Substitution):
            offset = float(self.__offset.perform(context))
        else:
            offset = self.__offset
        return f'{number + offset}'
