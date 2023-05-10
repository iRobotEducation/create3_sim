from typing import Union

from launch import LaunchContext, SomeSubstitutionsType, Substitution


class GetNamespacedName(Substitution):
    def __init__(
            self,
            namespace: Union[SomeSubstitutionsType, str],
            name: Union[SomeSubstitutionsType, str]
    ) -> None:
        self.__namespace = namespace
        self.__name = name

    def perform(
            self,
            context: LaunchContext = None,
    ) -> str:
        if isinstance(self.__namespace, Substitution):
            namespace = str(self.__namespace.perform(context))
        else:
            namespace = str(self.__namespace)

        if isinstance(self.__name, Substitution):
            name = str(self.__name.perform(context))
        else:
            name = str(self.__name)

        if namespace == '':
            namespaced_name = name
        else:
            namespaced_name = namespace + '/' + name
        return namespaced_name
